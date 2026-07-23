# Camera And Transforms

Camera adapters:

- `ExternalFixedCamera`: ROS subscriber for tabletop RealSense D555 namespace `table_camera`.
- `WristCamera`: future PiPER-on-Bunker adapter.
- `MockCamera`: offline development.
- `ReplayCamera`: reproduces fixture inputs without hardware.

Do not execute physical motion until camera-to-base calibration is real and validated.

Implemented topics:

- `/table_camera/color/image_raw`
- `/table_camera/aligned_depth_to_color/image_raw`
- `/table_camera/color/camera_info`

Target detection:

- ArUco marker detection with OpenCV.
- Aligned depth median from a small valid pixel neighborhood.
- Rejection of zero, invalid, non-finite, and out-of-range depth.
- No 3D target is claimed from only a 2D pixel.

Transform path:

- `StaticTransform` supports explicit `table_camera_color_optical_frame` to PiPER planning-frame transforms.
- Manipulation is refused when no valid base-frame target exists.
- No live camera-to-PiPER transform was verified during the 2026-07-23 audit.
