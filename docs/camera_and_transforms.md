# Camera And Transforms

Camera adapters:

- `ExternalFixedCamera`: current tabletop RealSense D555, namespace `table_camera`.
- `WristCamera`: future PiPER-on-Bunker adapter.
- `MockCamera`: offline development.
- `ReplayCamera`: reproduces fixture inputs without hardware.

Do not execute physical motion until camera-to-base calibration is real and validated.
