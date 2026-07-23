from __future__ import annotations

from typing import Optional

import math

from piper_on_bunker.models import Observation, Pose, Target


class MarkerDetector:
    def __init__(self, marker_id: Optional[int] = None, min_depth_m: float = 0.05, max_depth_m: float = 1.5) -> None:
        self.marker_id = marker_id
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m

    def detect(self, observation: Observation, label: str) -> Optional[Target]:
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            raise RuntimeError("MarkerDetector requires OpenCV and NumPy") from exc
        color = observation.metadata.get("_color_image")
        depth = observation.metadata.get("_depth_image")
        camera_matrix = observation.metadata.get("camera_matrix")
        if color is None or depth is None or not camera_matrix:
            return None

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        params = aruco.DetectorParameters()
        if hasattr(aruco, "ArucoDetector"):
            corners, ids, _ = aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None or len(ids) == 0:
            return None

        selected = 0
        flat_ids = [int(value[0]) for value in ids]
        if self.marker_id is not None:
            if self.marker_id not in flat_ids:
                return None
            selected = flat_ids.index(self.marker_id)
        pts = corners[selected][0]
        cx = int(round(float(pts[:, 0].mean())))
        cy = int(round(float(pts[:, 1].mean())))
        depth_m = self._median_depth_m(depth, cx, cy)
        if depth_m is None:
            return None
        fx, fy, ppx, ppy = float(camera_matrix[0]), float(camera_matrix[4]), float(camera_matrix[2]), float(camera_matrix[5])
        x = (cx - ppx) * depth_m / fx
        y = (cy - ppy) * depth_m / fy
        if not all(math.isfinite(value) for value in (x, y, depth_m)):
            return None
        perimeter = float(cv2.arcLength(pts.astype("float32"), True))
        confidence = max(0.0, min(1.0, perimeter / 400.0))
        pose = Pose(x=x, y=y, z=depth_m, frame_id=observation.frame_id)
        return Target(label=label, confidence=confidence, pixel=(cx, cy), camera_pose=pose, base_pose=None)

    def _median_depth_m(self, depth, cx: int, cy: int) -> Optional[float]:
        import numpy as np

        height, width = depth.shape[:2]
        x0, x1 = max(0, cx - 2), min(width, cx + 3)
        y0, y1 = max(0, cy - 2), min(height, cy + 3)
        patch = depth[y0:y1, x0:x1].astype("float64")
        values = patch[np.isfinite(patch) & (patch > 0)]
        if values.size == 0:
            return None
        median = float(np.median(values))
        if median > 10.0:
            median /= 1000.0
        if median < self.min_depth_m or median > self.max_depth_m:
            return None
        return median
