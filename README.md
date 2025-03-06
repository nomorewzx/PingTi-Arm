# PingTi Arm - A Low-Cost Robotic Arm with Human Arm Length 🤖💪

<center> <a href=./docs/README_CN.md> 中文版 </a> </center>

## What is PingTi Arm? 🤔

**PingTi Arm** is an open-source robotic arm designed based on the [SO-100 arm](https://github.com/TheRobotStudio/SO-ARM100) and is compatible with [Lerobot](https://github.com/huggingface/lerobot). The name "PingTi" comes from the Chinese pinyin "平替" (píng tì), which means "affordable substitute"

## Features 
- **Human arm scale**: 600mm arm span (excluding the end effector).
- **Reasonable payload**: Supports up to 500g at maximum arm span.
- **Cost Effective**: 
  - A single PingTi arm costs ~ USD 261, 
  - A Lead-Follower setup (PingTi arm as the follower, SO-100 as the leader) costs ~ USD 390.
  - If you already own an SO-100 follower arm (12V, 30kg.cm servos), you only need 2 additional servos and a USB drive board (~$157) to assemble the PingTi arm.


## Docs
- [Bills of Materials List](./docs/BOM_List.md) ｜ [Bills of Materials List 中文版](./docs/BOM_List_CN.md)
- [3D Printing](./docs/3d_print.md)
- [Assembly tutorial](./docs/Assembe_tutorial.md)
- [Example of run teleoperation and other tasks via `pingti_lerobot_bridge`](https://github.com/nomorewzx/pingti_lerobot_bridge)
- **(TODO)**: PingTi Arm on [LeKiwi base](https://github.com/SIGRobotics-UIUC/LeKiwi)
- **(TODO)**: [URDF for Simulation](./docs/URDF_SIM.md)

![The Right View of PingTi Arm](./drawings/PingTi_Arm_RightView_V1_20250218.jpg)

## Recommendation for Beginners

- I strongly recommend you to start with [SO-100](https://github.com/TheRobotStudio/SO-ARM100) and come to PingTi Arm if you are interested in:
    - a longer arm
    - reasonable payload capacity

- As stated before, if you assemble SO-100 with the `12V 30KG.cm` servos, you only need buy 2 more servos and 1 more USB control board to onboard PingTi arm. There is no waste of money

## Caution⚠️
- Avoid sudden movement and stop with heavy payload (e.g. 500g)
- Risk of shoulder joint damage if these conditions are not considered.
- Consider to increase the **infill density** when print the links if you would like to frequently use with heavy payloads


## Acknowledgement

- [SO-100](https://github.com/TheRobotStudio/SO-ARM100)
- [Lerobot](https://github.com/huggingface/lerobot)
- [LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi)