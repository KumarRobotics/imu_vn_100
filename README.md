[![Build Status](https://travis-ci.org/KumarRobotics/imu_vn_100.svg?branch=master)](https://travis-ci.org/KumarRobotics/imu\_vn\_100)

# imu\_vn\_100

![Picture of IMU](https://www.vectornav.com/images/default-source/products/vn-100-red-chip_right.png?sfvrsn=2302aad6_8)

The `imu_vn_100` package is a Linux ROS 2 driver for VN-100 Rugged IMU of [VECTORNAV](http://www.vectornav.com/). The package is developed based on the official [SDK v0.3.2](http://www.vectornav.com/support/downloads) for Linux. The user manual for the device can be found [here](http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/UM001.pdf?sfvrsn=10).

The package is tested on Ubuntu 18.04 with ROS 2 Dashing.

Note that the official SDK is modified within this package due to some bugs or unsupported features. Please carefully update the SDK, since the new SDK may not work with the provided software.

## License

* The license for the official SDK is the MIT license which is included in `vncpplib/LICENSE.txt`
* The license for the other code is Apache 2.0.

## Compiling

This is an Ament package. After cloning the package to your workspace, the normal procedure for compiling an ament package will work.

```
cd your_work_space
colcon build --packages-select imu_vn_100 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage

**Parameters**

`port` (`string`, `default: /dev/ttyUSB0`)

Port which the device is connected to. This can be checked by running `dmesg` right after plugging the device in.

`frame_id` (`string`, `default: imu_link`)

The frame ID entry for the sent messages.  This conforms to https://www.ros.org/reps/rep-0145.html .

`imu_rate` (`int`, `default: 200`)

The rate of the IMU data.

`initial_baudrate` (`int`, `default: 115200`)

The baud rate to use to initially connect to the serial port.  Out of the box, the device will use 115200 (the default).  However, it is possible to change the default baud rate permanently on the device using the `imu_vn100_flash_baud_rate` utility.  For maximum reliability, this should be set to the same as the `baudrate` below (think about the case where the node crashes; it got set to the `baudrate` value, but if we try to reconnect at a different rate it will fail).

`baudrate` (`int`, `default: 921600`)

The baud rate of the serial port. The available baud rates can be checked in the user manual. It is suggested that the baud rate is kept at `921600` to ensure high frequency transmission. The device will send `permission denied` error code if the baud rate cannot support the desired data package at the desired frequency.

`axes_convention` (`string`, `default: NED`)

The convention to use when publishing the axes.  The default is NED (which is what the device reports), but ENU is also supported.

`linear_acceleration_stddev` (`float`, `default: 0.0502828038`)

The standard deviation for linear acceleration to be reported when publishing data.

`angular_velocity_stddev` (`float`, `default: 0.04757054731142477`)

The standard deviation for angular velocity to be reported when publishing data.

`magnetic_field_stddev` (`float`, `default: 0.39636e-6`)

The standard deviation for the magnetic field to be reported when publishing data.

```
enable_mag  (bool, default: true)
enable_pres (bool, default: true)
enable_temp  (bool, default: true)
enable_rpy  (bool, default: false)
```

Enable other possible messages that the driver is available to send. Note that the frequency of the data for each of these messages will be the same as the IMU data, if the topic is enabled.

`sync_rate` (`int`, `default: 20`)

The rate of the sync out trigger signal. Note that the actual rate may not exactly match the provided rate, unless the provided rate is a divisor of 800. When `sync_rate` <= 0, it is disabled.

`sync_pulse_width_us` (`int`, `default: 1000`)

The width of the sync out pulse trigger signal, in microseconds.

`binary_output` (`boolean`, `default: true`)

Use binary protocol for receiving messages instead of ASCII.

`binary_async_mode` (`int`, `default: 2`)

Set serial port for binary messages to one of:

* `1` - Serial port 1 (this should be used if using the VN100-T Rugged with a USB Cable)
* `2` - Serial port 2

`imu_compensated` (`boolean`, `default: false`)

Use *compensated* IMU measurements (i.e. angular velocity, linear acceleration, magnetic field).

`vpe.enable` (`boolean`, `default: true`)

Use Vector Processing Engine.

`vpe.heading_mode` (`int`, `default: 1`)

Set VPE heading mode to one of:

* `0` - Absolute
* `1` - Relative
* `2` - Indoor

`vpe.filtering_mode` (`int`, `default: 1`)

Set VPE filtering mode to one of:

* `0` - Off
* `1` - *MODE 1*

`vpe.tuning_mode` (`int`, `default: 1`)

Set VPE tuning mode to one of:

* `0` - Off
* `1` - *MODE 1*

`vpe.mag_tuning.base_tuning.x` (`float`, `default: 4.0`)

The level of confidence in the magnetometer X-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.mag_tuning.base_tuning.y` (`float`, `default: 4.0`)

The level of confidence in the magnetometer Y-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.mag_tuning.base_tuning.z` (`float`, `default: 4.0`)

The level of confidence in the magnetometer Z-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.mag_tuning.adaptive_tuning.x` (`float`, `default: 5.0`)

The amount of adaptive tuning to apply to the magnetometer X-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.mag_tuning.adaptive_tuning.y` (`float`, `default: 5.0`)

The amount of adaptive tuning to apply to the magnetometer Y-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.mag_tuning.adaptive_tuning.z` (`float`, `default: 5.0`)

The amount of adaptive tuning to apply to the magnetometer Z-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.mag_tuning.adaptive_filtering.x` (`float`, `default: 5.5`)

The amount of adaptive filtering to apply to the magnetometer X-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`vpe.mag_tuning.adaptive_filtering.y` (`float`, `default: 5.5`)

The amount of adaptive filtering to apply to the magnetometer Y-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`vpe.mag_tuning.adaptive_filtering.z` (`float`, `default: 5.5`)

The amount of adaptive filtering to apply to the magnetometer Z-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`vpe.accel_tuning.base_tuning.x` (`float`, `default: 5.0`)

The level of confidence in the accelerometer X-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.accel_tuning.base_tuning.y` (`float`, `default: 5.0`)

The level of confidence in the accelerometer Y-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.accel_tuning.base_tuning.z` (`float`, `default: 5.0`)

The level of confidence in the accelerometer Z-axis when no disturbance is present.  Must be between 0.0 and 10.0 (higher is more confident).

`vpe.accel_tuning.adaptive_tuning.x` (`float`, `default: 3.0`)

The amount of adaptive tuning to apply to the accelerometer X-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.accel_tuning.adaptive_tuning.y` (`float`, `default: 3.0`)

The amount of adaptive tuning to apply to the accelerometer Y-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.accel_tuning.adaptive_tuning.z` (`float`, `default: 3.0`)

The amount of adaptive tuning to apply to the accelerometer Z-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher applies more tuning).

`vpe.accel_tuning.adaptive_filtering.x` (`float`, `default: 4.0`)

The amount of adaptive filtering to apply to the accelerometer X-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`vpe.accel_tuning.adaptive_filtering.y` (`float`, `default: 4.0`)

The amount of adaptive filtering to apply to the accelerometer Y-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`vpe.accel_tuning.adaptive_filtering.z` (`float`, `default: 4.0`)

The amount of adaptive filtering to apply to the accelerometer Z-axis.  0.0 disables.  Must be between 0.0 and 10.0 (higher levels do more filtering, at the expense of adding delay to the data).

`hsi.mode` (`int`, `default: 1`)

`hsi.output` (`int`, `default: 3`)

`hsi.converge_rate` (`int`, `default: 5`)

`ref.use_models` (`boolean`, `default: true`)

`ref.recalc_threshold_m` (`int`, `default: 10`)

`time_resynchronization_interval_ms` (`int`, `default: 5000`)

The amount of time in milliseconds between resynchronization of the time between the host computer and the IMU.  Lower values will resynchronize more often at the expense of causing time to jump around a bit more.  The default value is a good compromise between time drifting apart and time jumping around.

`callback_delta_epsilon_ms` (`int`, `default: 1`)

The amount of time slop in milliseconds to allow when trying to do a time resynchronization.  That is, we only try to do a synchronization when the data that arrives is not delayed significantly (this can happen during USB burst transfers, or due to unrelated operating system swapping).  The default value of 1 milliseconds works for most circumstances/

**Published Topics**

`imu/data` (`sensor_msgs/msg/Imu`)

The topic containing the angular velocity, linear acceleration, and orientation estimate.  If `imu_compensated` is `false`, the default, then the message contains the *uncompensated* (for the definition of UNCOMPENSATED, please refer to the [user manual](http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/UM001.pdf?sfvrsn=10)) angular velocity and linear acceleration. Otherwise both are *compensated*.

`imu/data_raw` (`sensor_msgs/msg/Imu`)

The topic containing the angular velocity and linear acceleration (no orientation estimate is provided).  If `imu_compensated` is `false`, the default, then the message contains the *uncompensated* (for the definition of UNCOMPENSATED, please refer to the [user manual](http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/UM001.pdf?sfvrsn=10)) angular velocity and linear acceleration. Otherwise both are *compensated*.

`imu/mag` (`sensor_msgs/msg/MagneticField`)

Magnetic field. If `imu_compensated` is `false` then it is *uncompensated* otherwise it is *compensated*.

`imu/fluid_pressure` (`sensor_msgs/msg/FluidPressure`)

Pressure.

`imu/temperature` (`sensor_msgs/msg/Temperature`)

Temperature in degree Celsius

`imu/rpy` (`geometry_msgs/msg/Vector3Stamped`)

Estimated roll (`x`), pitch (`y`) and yaw (`z`) angles in radians given as a 3,2,1 Euler angle rotation sequence describing the orientation of the sensor with respect to the inertial North East Down (NED) frame.

## FAQ

1. Why can't the driver open my device?\
Make sure you have ownership of the device in `/dev`.

2. Why are there permission errors during the initialization process of the driver?\
Most often, this is because the baud rate you set does not match the package size to be received. Try increasing the baud rate.

3. Why is the IMU data output rate much lower than what is set?\
This may be due to a recent change in the FTDI USB-Serial driver in the Linux kernel, the following shell script might help:
    ```bash
    # Reduce latency in the FTDI serial-USB kernel driver to 1ms
    # This is required due to https://github.com/torvalds/linux/commit/c6dce262
    for file in $(ls /sys/bus/usb-serial/devices/); do
      value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
      if [ $value -gt 1 ]; then
        echo "Setting low_latency mode for $file"
        sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
      fi
    done
    ```
4.  Why do I get 'VN: Permission denied' even after changing the permissions of the dev device?
When using the VN-100T with a USB cable, only serial device 1 is configured, so using the default
`binary_async_mode` of 2 will cause this error.  Set the `binary_async_mode` to `1` to make this work.

## Bugs or Features Required of the Official SDK

* [Solved] Define macros for options to set synchronization register and communication control register. (vndevice.h)

* [Solved] Pressure entry is not properly parsed and overwrites temperature in `vndevice_processAsyncData` when the async data output type is set to `VNASYNC_VNIMU`. (vndevice.c)

* [Solved] Synchronization time or count is not parsed in `vndevice_processAsyncData`. (vndevice.c)

* [Solved] The setReferenceVectorConfiguration sent the wrong command to the device.  (vndevice.c)

* [Solved] Bug parsing the syncInCnt during processAsyncData calls.  (vndevice.c)

* Angular velocity and linear acceleration are flipped when the device is set to binary output using `BG1_IMU`.

* Orientation and uncompensated IMU measurements cannot be acquired within a single setting using async output.
