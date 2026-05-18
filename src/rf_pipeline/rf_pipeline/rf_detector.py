#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import torch
import torch.nn.functional as F
import torchvision

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from rf_msgs.msg import IqFrame, StftFrame, RfDetection, RfDetectionArray
from sensor_msgs.msg import Image

LABELS = ["Chirp", "Wideband", "Narrowband", "FHSS"]
IMG_SIZE = 640


class RfDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("rf_detector_node")

        self._buffer_size = max(int(self.declare_parameter("buffer_size", 100000).value), 2)
        self._win_size    = max(int(self.declare_parameter("win_size", self._buffer_size // 4).value), 2)
        self._nfft        = max(int(self.declare_parameter("nfft", self._win_size).value), 2)
        self._hop         = max(int(self.declare_parameter("hop", self._win_size // 2).value), 1)
        self._conf_thresh = float(self.declare_parameter("conf_thresh", 0.5).value)

        default_model = str(
            Path(get_package_share_directory("rf_pipeline")) / "resource" / "best.torchscript"
        )
        self._model_path = str(self.declare_parameter("model_path", default_model).value)

        if self._buffer_size < 2:
            self.get_logger().warning(f"Buffer size too small ({self._buffer_size}), setting to 2")
            self._buffer_size = 2
        if self._win_size > self._buffer_size:
            self.get_logger().warning(f"Window size ({self._win_size}) larger than buffer, setting to {self._buffer_size}")
            self._win_size = self._buffer_size
        if self._nfft < self._win_size:
            self.get_logger().warning(f"FFT size ({self._nfft}) smaller than window, setting to {self._win_size}")
            self._nfft = self._win_size
        if self._hop >= self._win_size:
            self.get_logger().warning(f"Hop size ({self._hop}) should be smaller than window, setting to {self._win_size // 2}")
            self._hop = self._win_size - 1

        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Symmetric Hann window on GPU
        hann_cpu = torch.hann_window(self._win_size, periodic=False, dtype=torch.float32)
        self._win_sum_sq = float(hann_cpu.pow(2).sum().item())
        self._hann = hann_cpu.to(self._device)

        self.get_logger().info(f"Loading TorchScript model: {self._model_path}")
        self._model = torch.jit.load(self._model_path, map_location=self._device)
        self._model.eval()

        # State
        self._fs_hz: float | None = None
        self._cf_hz: float | None = None
        self._nsamples = 0
        self._iq_buffer = torch.zeros(self._buffer_size * 2, dtype=torch.float32)
        self._freq_ghz: list[float] = []
        self._time_s:   list[float] = []
        self._current_stamp = None

        self._det_pub  = self.create_publisher(RfDetectionArray, "detections", 10)
        self._stft_pub = self.create_publisher(StftFrame, "stft", 10)
        self._img_pub  = self.create_publisher(Image, "stft_image", 10)

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(IqFrame, "iq", self._on_iq, qos)

        self.get_logger().info(
            f"RfDetectorNode ready: buf={self._buffer_size}  win={self._win_size}  "
            f"fft={self._nfft}  hop={self._hop}  conf={self._conf_thresh:.2f}"
        )

    # ── IQ accumulation ───────────────────────────────────────────────────────

    def _on_iq(self, msg: IqFrame) -> None:
        if self._fs_hz != msg.fs_hz:
            self._fs_hz    = msg.fs_hz
            self._nsamples = 0
            self._update_time_s()
        if self._cf_hz != msg.fc_hz:
            self._cf_hz = msg.fc_hz
            self._update_freq_ghz()

        raw = msg.data
        if len(raw) % 2 != 0:
            self.get_logger().warning(f"Dropping IQ frame: odd element count ({len(raw)})")
            return

        self._current_stamp = msg.stamp
        iq_len       = len(raw) // 2
        samples_read = 0
        while samples_read < iq_len:
            to_read = min(self._buffer_size - self._nsamples, iq_len - samples_read)
            src = samples_read * 2
            dst = self._nsamples * 2
            self._iq_buffer[dst : dst + to_read * 2] = torch.as_tensor(
                raw[src : src + to_read * 2], dtype=torch.float32
            )
            self._nsamples    += to_read
            samples_read      += to_read
            if self._nsamples == self._buffer_size:
                self._process_buffer()
                self._nsamples = 0

    def _update_freq_ghz(self) -> None:
        k = np.arange(self._nfft, dtype=np.float64)
        self._freq_ghz = (((k - self._nfft / 2) * self._fs_hz / self._nfft + self._cf_hz) / 1e9).tolist()

    def _update_time_s(self) -> None:
        n_frames = 1 + (self._buffer_size - self._win_size) // self._hop
        t = np.arange(n_frames, dtype=np.float64)
        self._time_s = (t * self._hop / self._fs_hz + self._win_size / (2 * self._fs_hz)).tolist()

    # ── GPU pipeline ──────────────────────────────────────────────────────────

    def _process_buffer(self) -> None:
        if not self._fs_hz:
            return

        # ① CPU → GPU: interleaved float IQ → complex64
        iq_cplx = torch.view_as_complex(
            self._iq_buffer.view(self._buffer_size, 2).contiguous()
        ).to(self._device)

        # ② STFT → [nfft, n_frames] complex64
        spectrum = torch.stft(
            iq_cplx,
            n_fft=self._nfft,
            hop_length=self._hop,
            win_length=self._win_size,
            window=self._hann,
            normalized=False,
            onesided=False,
            return_complex=True,
        )

        # ③ fftshift + PSD + dB → [nfft, n_frames] float32
        scale   = 1.0 / (self._fs_hz * self._win_sum_sq)
        shifted = torch.roll(spectrum, self._nfft // 2, dims=0)
        psd_db  = torch.log10(shifted.abs().pow(2).mul(scale).clamp_min(1e-30)).mul(10.0)

        # ④ Conditional StftFrame publish
        if self._stft_pub.get_subscription_count() > 0:
            self._publish_stft_frame(psd_db)

        # ⑤ Percentile normalization (kthvalue is 1-indexed)
        flat    = psd_db.flatten()
        n_total = flat.numel()
        lo_idx  = max(0,          round(0.010 * (n_total - 1)))
        hi_idx  = min(n_total - 1, round(0.995 * (n_total - 1)))
        lo = torch.kthvalue(flat, lo_idx + 1).values.item()
        hi = torch.kthvalue(flat, hi_idx + 1).values.item()
        span = (hi - lo) if hi > lo else 1.0
        norm = ((psd_db - lo) / span).clamp(0.0, 1.0)

        # ⑥ Resize to [1, 3, 640, 640]
        # psd_db: [nfft, n_frames] = [freq, time] → transpose → [time, freq]
        img = norm.t().unsqueeze(0).unsqueeze(0)   # [1, 1, n_frames, nfft]
        img_640 = F.interpolate(img, size=(IMG_SIZE, IMG_SIZE), mode="bilinear", align_corners=False)
        img_3ch = img_640.repeat(1, 3, 1, 1)       # [1, 3, 640, 640]

        # ⑦ Conditional Image publish
        if self._img_pub.get_subscription_count() > 0:
            self._publish_image(img_3ch)

        # ⑧ Inference
        with torch.no_grad():
            out = self._model.forward(img_3ch)
        dets = self._parse_dets(out)   # [N, 6] = [x1, y1, x2, y2, conf, cls]

        # ⑨ Confidence filter + coordinate transform on GPU
        if dets.shape[0] > 0:
            dets = dets[dets[:, 4] >= self._conf_thresh]

        if dets.shape[0] > 0 and len(self._freq_ghz) >= 2 and len(self._time_s) >= 2:
            f_min  = self._freq_ghz[0]
            f_span = self._freq_ghz[-1] - f_min
            t_min  = self._time_s[0]
            t_span = self._time_s[-1] - t_min
            scale_t  = torch.tensor(
                [f_span / IMG_SIZE, t_span / IMG_SIZE, f_span / IMG_SIZE, t_span / IMG_SIZE, 1.0, 1.0],
                dtype=torch.float32, device=dets.device,
            )
            offset_t = torch.tensor(
                [f_min, t_min, f_min, t_min, 0.0, 0.0],
                dtype=torch.float32, device=dets.device,
            )
            dets = dets * scale_t + offset_t   # broadcast [M,6] * [6] + [6]

        # ⑩ GPU → CPU → publish
        self._publish_detections(dets)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_stft_frame(self, psd_db: torch.Tensor) -> None:
        # psd_db: [nfft, n_frames] float32 on GPU
        # StftFrame waterfall_db: row-major [time][freq], float64
        wf = psd_db.t().contiguous().double().cpu()   # [n_frames, nfft]

        msg = StftFrame()
        msg.stamp      = self._current_stamp
        msg.fs_hz      = float(self._fs_hz)
        msg.num_frames = wf.shape[0]
        msg.fft_size   = wf.shape[1]
        msg.waterfall_db = wf.flatten().tolist()
        msg.freq_ghz   = self._freq_ghz
        msg.time_s     = self._time_s
        self._stft_pub.publish(msg)

    def _publish_image(self, img_3ch: torch.Tensor) -> None:
        # img_3ch: [1, 3, 640, 640] float32 on GPU, values in [0, 1]
        img_cpu = (
            img_3ch.squeeze(0)           # [3, 640, 640]
                   .permute(1, 2, 0)     # [640, 640, 3] HWC
                   .mul(255.0)
                   .clamp(0.0, 255.0)
                   .byte()
                   .contiguous()
                   .cpu()
        )
        msg = Image()
        msg.header.stamp    = self._current_stamp
        msg.header.frame_id = "stft"
        msg.height          = IMG_SIZE
        msg.width           = IMG_SIZE
        msg.encoding        = "rgb8"
        msg.is_bigendian    = False
        msg.step            = IMG_SIZE * 3
        msg.data            = img_cpu.numpy().tobytes()
        self._img_pub.publish(msg)

    def _publish_detections(self, dets: torch.Tensor) -> None:
        arr = RfDetectionArray()
        arr.stamp = self._current_stamp

        if dets.dim() == 2 and dets.shape[0] > 0:
            cpu = dets.cpu()
            for row in cpu:
                x1, y1, x2, y2, conf, cls_f = row.tolist()
                cls = int(cls_f)
                det = RfDetection()
                det.kind         = LABELS[cls] if 0 <= cls < len(LABELS) else str(cls)
                det.confidence   = float(conf)
                det.f_lo_ghz     = float(x1)
                det.t_lo_s       = float(y1)
                det.f_hi_ghz     = float(x2)
                det.t_hi_s       = float(y2)
                det.f_center_ghz = (float(x1) + float(x2)) * 0.5
                det.bw_ghz       = abs(float(x2) - float(x1))
                arr.detections.append(det)

        self._det_pub.publish(arr)

    # ── Output parsing ────────────────────────────────────────────────────────

    @staticmethod
    def _parse_dets(val) -> torch.Tensor:
        if isinstance(val, torch.Tensor):
            return val.squeeze(0) if val.dim() == 3 else val
        if isinstance(val, (list, tuple)) and len(val) > 0:
            first = val[0]
            return RfDetectorNode._parse_dets(first)
        return torch.zeros((0, 6))