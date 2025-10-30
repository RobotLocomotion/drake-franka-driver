# Panda Arm - Franka Emika
This repository contains the application code used to communicate with the Franka Emika arm from Drake.

## Software Versions

This driver supports both FR3 (newer) and FE3 (older) Franka robots:

- **Drake**: v1.45.0
- **libfranka v5** (~0.15.0): For FR3 robots with firmware v5.x
- **libfranka v4** (0.9.0): For FE3 robots with firmware v4.x
- **Pinocchio**: 2.6.21 (required for libfranka v5 dynamics)
- **LCM**: Latest from main branch 

## Building the driver
Before you begin, install the prerequisites:
```
sudo apt install libgflags-dev libpoco-dev
```

To build, run `bazel build //...`.  This will output two versions of
the driver: `bazel-bin/franka-driver/franka_driver_v4` and
`bazel-bin/franka-driver/franka_driver_v5`.

Use `franka_driver_v5` for the FR-3 robots.

# Running the driver
To run the driver in position mode, use the following command:
```
    bazel-bin/franka-driver/franka_driver_v5 --expire_sec=0.05 --robot_ip_address=<ip-addr> --control_mode=position
```

To run the driver in torque mode, use the following command:
```
bazel-bin/franka-driver/franka_driver_v5 --use_mbp --expire_sec=0.05 --robot_ip_address=<ip-addr> --control_mode=torque
```

To add a gripper, update the `models/add_franka_control.yaml` with the respective gripper.

## Links

* [Website](https://www.franka.de/technology)
* [Franka Control Interface (FCI) Documentation](https://frankarobotics.github.io/docs/)
* [`libfranka` API Docs](https://frankarobotics.github.io/libfranka/)
* [Panda datasheet](https://s3-eu-central-1.amazonaws.com/franka-de-uploads/uploads/Datasheet-EN.pdf)

## Notes

* Be sure to read the manual, FCI docs, etc.
    * The FCI docs have excellent instructions for getting your network
    interface on Ubuntu set up.
* There are two very distinct versions of Pandas: the FE3 (older) and FR3 (newer)
* There are three versions to think about: firmware on the robot (we use
`v4.x` and `v5.x`), `libfranka` driver software on the host PC
(0.9.0 for `v4.x` FE3, and ~0.15.0 for `v5.x` FR3), and the robot model itself.
    * You should see suffixes like `_v4` and `_v5` for relevant
    libraries and binaries throughout this codebase.
    * If you use the wrong driver, you should get an error like:

            terminate called after throwing an instance of 'franka::IncompatibleVersionException'
            what():  libfranka: Incompatible library version (server version: 4, library version: 5). Please check https://frankaemika.github.io for Panda system updates or use a different version of libfranka.

