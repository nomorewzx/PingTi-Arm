# PingTi Arm - A Low-Cost Robotic Arm with Human Arm Length 🤖💪


### Why PingTi Arm? 🤔

`SO-100` is a great low-cost open source robotic arm and Lerobot is a fantastic AI for Robotics open source project. But `SO-100` has below limitations:

- **Short arm span**, which makes it almost a toy and hardly to do practical work with its span length 
- **Light payload**, which makes it really hard to do a simple work like picking a bootle of water

In short, we need to buy long and high payload robotic arms in order to seriously study the state-of-the-art AI for robotics (pi0, OpenVLA etc.), and these robotic arms are usually expensive.

### What is PingTi Arm ✨

A low-cost open source arm based on `SO-100` and addresses its 2 limitations by introducing:

- **Human arm length 🦾**. By simply extending the lower arm and upper arm of `SO-100`, the robotic arm now has around 65cm span length.
- **Reasonable payload 💪**. In the key heavy-lifting joints `shoulder_lift` and `wrist_lift`, I use 2 powerful servos to drive one single joint, providing a higher total torque and more stable motion control.

![The Right View of PingTi Arm](./drawings/PingTi_Arm_RightView_V1_20250216.jpg)

