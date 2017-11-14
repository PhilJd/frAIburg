## FrAIburg Realsense Filter

This is a modified version of the original realsense filter by AUDI.

Intel's realsense lib
matches [63 pixels](https://github.com/IntelRealSense/librealsense/blob/legacy/doc/camera_specs.md#r200-notes).
Therefore decreasing the depth resolution to 320x240
decreases the blind distance from 72 to 32 cm. The original filter
crashed at this resolution. This filter only allows a depth resolution of 320x240.
It also contains minor performance improvements when no display is connected.

### ToDo:
- Remove Raw Infrared streams if we don't use them