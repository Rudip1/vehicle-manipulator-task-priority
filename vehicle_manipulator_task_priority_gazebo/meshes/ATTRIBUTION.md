# Mesh sources

The visual meshes in this folder are copied unchanged from public repositories. They are used for appearance only;
collisions are simple shapes and the kinematics come from the measured link lengths
(`vehicle_manipulator_task_priority/include/vehicle_manipulator_task_priority/swift_pro.hpp`).

| Files | Source | Licence |
|---|---|---|
| `kobuki/main_body.dae`, `kobuki/wheel.dae`, `kobuki/images/*.jpg` | [yujinrobot/kobuki](https://github.com/yujinrobot/kobuki) @ `4f7f1beea87a`, `kobuki_description/meshes/` | BSD, as declared in that package's `package.xml` (Yujin Robot) |
| `swiftpro/{Base,Link1,Link2,Link3,Suction}.STL` | [uArm-Developer/RosForSwiftAndSwiftPro](https://github.com/uArm-Developer/RosForSwiftAndSwiftPro) @ `a735297d586a`, `swiftpro/urdf/pro_links/` | No licence is stated in the source repository (its `package.xml` says `TODO`). The meshes are © UFACTORY and are reproduced here with attribution for a non-commercial research demonstration. |

The parallelogram bars of the uArm (`Link4`-`Link7`, `Link9`) are not used: the arm is modelled as a serial chain whose
wrist joint keeps the end effector level, which is what the parallelogram does mechanically.
