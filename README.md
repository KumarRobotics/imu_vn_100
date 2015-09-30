# imu_vn_100

![Picture of IMU](http://www.vectornav.com/images/default-source/products/vn-100-rugged.png?sfvrsn=2)

The `imu_vn_100` package is a linux ROS driver for VN-100 Rugged IMU of [VECTORNAV](http://www.vectornav.com/). The package is developed based on the offical [SDK v0.3.2](http://www.vectornav.com/support/downloads) for Linux.

The package is tested on Ubuntu 14.04 with ROS indigo.

Note that the offical SDK is modified within this package due to some bugs. Please carefully update the SDK. The new SDK may not work with the provided software.

## License
* The license for the offical SDK is the MIT license which is included in the `include/imu_vn_100/vncpplib`
* The license for the other codes is Apache 2.0 whenever not specified.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make --pkg imu_vn_100 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Example Usage

**Parameters**

`port` (`string`, `default: \tty\USB0`)

Port which the device connected to. This can be checked by command `dmesg`.

`frame_id` (`string`, `default: imu`)

The frame ID entry for the sent messages.

`baudrate` (`int`, `921600`)

The baud rate of the serial port. The available baud rates can be checked on the user manual. It is suggested that the baud rate is kept at `921600` to ensure the high frequency transimission. The device will send `permission denied` error code if the baud rate cannot support the desired data package.

`imu_rate` (`int`, `200`)

The rate of the IMU data.

```
enable_magnetic_field (bool, default: true)
enable_pressure       (bool, default: true)
enable_temperature    (bool, default: true)
```

Other possible messages that the driver is available to send. Note that the frequency of the data for each of these messages will be the same with the IMU data, if the topic is enabled.

`enable_sync_out` (`bool`, `true`)

Once set to true, the driver will record the time when a sync out trigger signal is sent. This is useful when the VN-100 IMU is to be synchronized with a camera.

`sync_out_rate` (`int`, `30`)

The rate of the sync out trigger signal. Note that the actual rate may not exactly match the provided rate, unless the provided rate is a divisor of 800.

## FAQ

## Bug Report

## Bug Trace of the Offical SDK

