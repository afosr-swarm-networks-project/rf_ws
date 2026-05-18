from __future__ import annotations

from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from rf_msgs.msg import RfDetection, RfDetectionArray, RfScore, RfScoreArray


class ScoreboardNode(Node):
    def __init__(self) -> None:
        super().__init__("scoreboard")

        self.declare_parameter("aggregation", "max")

        detections_topic = "detections"
        scores_topic = "scores"
        self._aggregation = str(self.get_parameter("aggregation").value)

        self._pub = self.create_publisher(RfScoreArray, scores_topic, 10)
        self._sub = self.create_subscription(
            RfDetectionArray, detections_topic, self._on_detections, 10
        )

        self.get_logger().info(
            "ScoreboardNode ready: "
            f"aggregation={self._aggregation} "
        )

    def _on_detections(self, msg: RfDetectionArray) -> None:

        scores_msg = RfScoreArray()
        scores_msg.stamp = msg.stamp

        agg = self._aggregation.lower()

        per_kind_detections: dict[str, list[float]] = {}
        for detection in msg.detections:
            value = float(detection.confidence)
            if detection.kind not in per_kind_detections:
                per_kind_detections[detection.kind] = []
            per_kind_detections[detection.kind].append(value)

        for kind, values in per_kind_detections.items():
            
            confidences = np.asarray(values, dtype=float)

            score = RfScore()
            score.kind = kind

            if agg == "max":
                score.value = float(np.max(confidences))
            elif agg == "mean":
                score.value = float(np.mean(confidences))
            elif agg == "sum":
                score.value = float(np.sum(confidences))
            elif agg == "sum_clip":
                score.value = float(np.clip(np.sum(confidences), 0.0, 1.0))
            else:
                self.get_logger().error(f"unsupported YOLO aggregation mode: {agg}")

            scores_msg.scores.append(score)

        self._pub.publish(scores_msg)
