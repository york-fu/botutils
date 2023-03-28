# botutils

## Summary

Some commonly used functions for robots.

Provide：

- performance measurement
- rate: Clock loop with wait
- timer
- lowpass fitler
- kalman filter
- fusion quaternion: Update quaternion based on gyroscope and accelerometer and magnetometer

## Requirements

### kalman filter module

- Eigen3

## Build

```
mkdir build
cd build
cmake ..
make -j`nproc`
```

## Test

```
cd build
./test/tests
```