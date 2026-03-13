# VB2 SDK

This branch contains the control and sensor SDK for the VB2 humanoid robot.

> **Note**: This main controller is strictly limited to managing **non-chassis components** (e.g., robotic arms, dexterous hands, head) and reading **upper-body sensors** (e.g., chest camera, wrist cameras). Chassis control is not included in this branch. The chassis control SDK is available here: [VMR-AMR SDK](https://gitee.com/VMR-Robotics-admin/vmr_-amr_-sdk)

## Control Module

The control module can be found in the `C++/control` directory. It handles the actuation and control of all supported upper-body hardware.

## Sensor Module

The sensor module can be found in the `C++/sensor` directory. It is responsible for acquiring data from the upper-body vision and sensor systems.
