# Export RGBD images from Kinect recording

## What's included

- Skip frames in the first second for rgb camera initialization
- Project depth map to rgb image coordinate system
- Center crop the RGBD image and export
    - Both rgb and depth image are named with its timestamp (us).
    - Depth image is stored with uint16 format, the unit is mm.
- Dump the IMU data
    - Format: acc_timestamp acc_x acc_y acc_z gyro_timestamp gyro_x gyro_y gyro_z
    - The units for timestamp, acc and gyro are us, m/s, m/s^2.

## Dependency

- Azure Kinect Sensor SDK
- OpenCV 4.7

## Build

```
mkdir build && cd build
cmake ..
make
```

## Usage
./exportRGBD path_to_recorded_mkv path_to_export_dir crop_width crop_height

