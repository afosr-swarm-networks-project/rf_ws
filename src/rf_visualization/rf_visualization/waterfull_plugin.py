from __future__ import annotations

import colorsys
import hashlib
from typing import Optional

import numpy as np
import pyqtgraph as pg

from python_qt_binding.QtCore import QTimer, Signal, Slot
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rqt_gui_py.plugin import Plugin

from rf_msgs.msg import RfDetectionArray, StftFrame


class WaterfullWidget(QWidget):
    _stft_ready = Signal(object)
    _det_ready  = Signal(object)

    def __init__(self, node) -> None:
        super().__init__()
        self._node = node

        self._stft_msg: Optional[StftFrame] = None
        self._detections_msg: Optional[RfDetectionArray] = None
        self._stft_sub = None
        self._det_sub = None
        self._has_image = False
        self._box_items:   list = []   # pg.PlotCurveItem per detection
        self._label_items: list = []   # pg.TextItem per detection
        self._class_visibility: dict[str, bool] = {}
        self._class_checkboxes: dict[str, QCheckBox] = {}
        self._confidence_threshold = 0.0
        self._updating_topics = False

        self._build_ui()

        self._stft_ready.connect(self._on_stft_ready)
        self._det_ready.connect(self._on_det_ready)

        self._topic_timer = QTimer(self)
        self._topic_timer.timeout.connect(self._refresh_topic_lists)
        self._topic_timer.start(1500)

        self._refresh_topic_lists()

    def shutdown(self) -> None:
        self._topic_timer.stop()
        self._unsubscribe_stft()
        self._unsubscribe_detections()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("STFT Topic:"))
        self._stft_combo = QComboBox()
        self._stft_combo.currentTextChanged.connect(self._on_stft_topic_changed)
        controls.addWidget(self._stft_combo, 2)

        controls.addWidget(QLabel("Detections Topic:"))
        self._det_combo = QComboBox()
        self._det_combo.currentTextChanged.connect(self._on_detections_topic_changed)
        controls.addWidget(self._det_combo, 2)

        refresh_button = QPushButton("Refresh Topics")
        refresh_button.clicked.connect(self._refresh_topic_lists)
        controls.addWidget(refresh_button)

        layout.addLayout(controls)

        class_controls = QHBoxLayout()
        class_controls.addWidget(QLabel("Show Classes:"))
        self._class_filter_widget = QWidget()
        self._class_filter_layout = QHBoxLayout(self._class_filter_widget)
        self._class_filter_layout.setContentsMargins(0, 0, 0, 0)
        self._class_filter_layout.setSpacing(8)
        self._class_filter_layout.addStretch()
        class_controls.addWidget(self._class_filter_widget, 1)
        class_controls.addWidget(QLabel("Min Confidence:"))
        self._confidence_spinbox = QDoubleSpinBox()
        self._confidence_spinbox.setRange(0.0, 1.0)
        self._confidence_spinbox.setSingleStep(0.05)
        self._confidence_spinbox.setDecimals(2)
        self._confidence_spinbox.setValue(self._confidence_threshold)
        self._confidence_spinbox.valueChanged.connect(self._on_confidence_threshold_changed)
        class_controls.addWidget(self._confidence_spinbox)
        layout.addLayout(class_controls)

        # ── pyqtgraph canvas ──────────────────────────────────────────────────
        pg.setConfigOptions(antialias=False)

        self._glw = pg.GraphicsLayoutWidget()
        self._plot = self._glw.addPlot(row=0, col=0)
        self._plot.setLabel("bottom", "Frequency (GHz)")
        self._plot.setLabel("left", "Time (s)")
        self._plot.setTitle("RF Waterfull Viewer")

        self._img_item = pg.ImageItem()
        cmap = pg.colormap.get("inferno")
        self._img_item.setColorMap(cmap)
        self._plot.addItem(self._img_item)

        self._cbar = pg.ColorBarItem(colorMap=cmap, label="STFT (dB)")
        self._cbar.setImageItem(self._img_item)
        self._glw.addItem(self._cbar, row=0, col=1)

        layout.addWidget(self._glw)

    # ── Topic management ──────────────────────────────────────────────────────

    def _refresh_topic_lists(self) -> None:
        topic_names_and_types = self._node.get_topic_names_and_types()
        stft_topics = sorted(
            name for name, types in topic_names_and_types
            if "rf_msgs/msg/StftFrame" in types
        )
        det_topics = sorted(
            name for name, types in topic_names_and_types
            if "rf_msgs/msg/RfDetectionArray" in types
        )

        self._updating_topics = True
        try:
            self._reset_combo(self._stft_combo, stft_topics)
            self._reset_combo(self._det_combo, det_topics)
        finally:
            self._updating_topics = False

        if self._stft_combo.count() > 0 and self._stft_sub is None:
            self._on_stft_topic_changed(self._stft_combo.currentText())
        if self._det_combo.count() > 0 and self._det_sub is None:
            self._on_detections_topic_changed(self._det_combo.currentText())

    def _reset_combo(self, combo: QComboBox, topics: list) -> None:
        current = combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        combo.addItems(topics)
        if current:
            idx = combo.findText(current)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _on_stft_topic_changed(self, topic: str) -> None:
        if self._updating_topics or not topic:
            return
        self._unsubscribe_stft()
        self._stft_msg = None
        self._stft_sub = self._node.create_subscription(
            StftFrame, topic, self._on_stft, 10
        )

    def _on_detections_topic_changed(self, topic: str) -> None:
        if self._updating_topics or not topic:
            return
        self._unsubscribe_detections()
        self._detections_msg = None
        self._sync_class_filters([])
        self._redraw_detections()
        self._det_sub = self._node.create_subscription(
            RfDetectionArray, topic, self._on_detections, 10
        )

    def _unsubscribe_stft(self) -> None:
        if self._stft_sub is not None:
            self._node.destroy_subscription(self._stft_sub)
            self._stft_sub = None

    def _unsubscribe_detections(self) -> None:
        if self._det_sub is not None:
            self._node.destroy_subscription(self._det_sub)
            self._det_sub = None

    # ── ROS callbacks (executor thread) ──────────────────────────────────────

    def _on_stft(self, msg: StftFrame) -> None:
        self._stft_ready.emit(msg)

    def _on_detections(self, msg: RfDetectionArray) -> None:
        self._det_ready.emit(msg)

    # ── Qt slots (GUI thread) ─────────────────────────────────────────────────

    @Slot(object)
    def _on_stft_ready(self, msg: StftFrame) -> None:
        self._stft_msg = msg
        self._redraw_waterfall()
        self._redraw_detections()

    @Slot(object)
    def _on_det_ready(self, msg: RfDetectionArray) -> None:
        self._detections_msg = msg
        self._sync_class_filters(sorted({det.kind for det in msg.detections if det.kind}))
        if not self._has_image:
            return
        self._redraw_detections()

    # ── Rendering ─────────────────────────────────────────────────────────────

    def _redraw_waterfall(self) -> None:
        msg = self._stft_msg
        if msg is None or msg.num_frames == 0 or msg.fft_size == 0:
            return

        try:
            sxx = np.asarray(msg.waterfall_db, dtype=np.float32).reshape(
                int(msg.num_frames), int(msg.fft_size)
            )
            f = np.asarray(msg.freq_ghz, dtype=np.float64)
            t = np.asarray(msg.time_s,   dtype=np.float64)
        except ValueError:
            return

        if f.size < 2 or t.size < 2:
            return

        f_min, f_max = float(f[0]), float(f[-1])
        t_min, t_max = float(t[0]), float(t[-1])

        lo = float(np.percentile(sxx, 1.0))
        hi = float(np.percentile(sxx, 99.5))

        # sxx: [n_frames, n_freq] = [time, freq]
        # ImageItem expects [x, y] = [freq, time], so transpose
        self._img_item.setImage(sxx.T, autoLevels=False, levels=[lo, hi])
        self._img_item.setRect(
            pg.QtCore.QRectF(f_min, t_min, f_max - f_min, t_max - t_min)
        )
        self._cbar.setLevels(low=lo, high=hi)

        self._plot.setTitle(
            f"RF Waterfull Viewer | {int(msg.fft_size)} bins x {int(msg.num_frames)} frames"
        )

        if not self._has_image:
            self._plot.setXRange(f_min, f_max, padding=0)
            self._plot.setYRange(t_min, t_max, padding=0)
            self._has_image = True

    def _redraw_detections(self) -> None:
        for item in self._box_items:
            self._plot.removeItem(item)
        self._box_items.clear()

        for item in self._label_items:
            self._plot.removeItem(item)
        self._label_items.clear()

        if self._detections_msg is None:
            return

        for det in self._detections_msg.detections:
            if det.confidence < self._confidence_threshold:
                continue
            if det.kind and not self._class_visibility.get(det.kind, True):
                continue

            color = self._color_for_kind(det.kind)

            box = pg.PlotCurveItem(
                x=[det.f_lo_ghz, det.f_hi_ghz, det.f_hi_ghz, det.f_lo_ghz, det.f_lo_ghz],
                y=[det.t_lo_s,   det.t_lo_s,   det.t_hi_s,   det.t_hi_s,   det.t_lo_s],
                pen=pg.mkPen(color, width=2),
            )
            self._plot.addItem(box)
            self._box_items.append(box)

            label = pg.TextItem(
                text=f"{det.kind} {det.confidence:.0%}",
                color=color,
                anchor=(0, 1),
                fill=pg.mkBrush(0, 0, 0, 115),
            )
            label.setPos(det.f_lo_ghz, det.t_hi_s)
            self._plot.addItem(label)
            self._label_items.append(label)

    def _sync_class_filters(self, kinds: list[str]) -> None:
        ordered_kinds = sorted(dict.fromkeys(kinds))
        current_kinds = set(self._class_checkboxes)
        next_kinds    = set(ordered_kinds)

        if current_kinds == next_kinds:
            return

        for kind in list(self._class_checkboxes):
            if kind in next_kinds:
                continue
            checkbox = self._class_checkboxes.pop(kind)
            self._class_filter_layout.removeWidget(checkbox)
            checkbox.deleteLater()
            self._class_visibility.pop(kind, None)

        for kind in ordered_kinds:
            if kind in self._class_checkboxes:
                continue
            checkbox = QCheckBox(kind)
            checkbox.setChecked(self._class_visibility.get(kind, True))
            checkbox.toggled.connect(
                lambda checked, class_name=kind: self._on_class_filter_toggled(class_name, checked)
            )
            checkbox.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
            insert_at = max(0, self._class_filter_layout.count() - 1)
            self._class_filter_layout.insertWidget(insert_at, checkbox)
            self._class_checkboxes[kind] = checkbox
            self._class_visibility[kind] = checkbox.isChecked()

    def _on_class_filter_toggled(self, kind: str, checked: bool) -> None:
        self._class_visibility[kind] = checked
        self._redraw_detections()

    def _on_confidence_threshold_changed(self, value: float) -> None:
        self._confidence_threshold = value
        self._redraw_detections()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _color_for_kind(self, kind: str) -> str:
        if not kind:
            return "#ffffff"
        digest = hashlib.md5(kind.encode("utf-8")).hexdigest()
        hue = (int(digest[:8], 16) % 360) / 360.0
        r, g, b = colorsys.hls_to_rgb(hue, 0.60, 0.85)
        return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


class WaterfullPlugin(Plugin):
    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("WaterfullPlugin")

        self._widget = WaterfullWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"RF Waterfull Viewer ({context.serial_number()})"
            )
        context.add_widget(self._widget)

    def shutdown_plugin(self) -> None:
        self._widget.shutdown()
