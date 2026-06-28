**!!! NO AI CODE USED HERE !!!**

 So, for my 2nd project, this implements a dynamic N-DOF planar serial manipulator simulation using the Cyclic Coordinate Descent (CCD) algorithm for real-time inverse kinematics. The solver iteratively minimizes the error between the end-effector and a target point in task space, employing adaptive convergence scaling to account for varying link leverage. The workspace is constrained to the manipulator's reachable radius, and singularity protection prevents numerical instability at degenerate configurations.

 If we remove the scary technical jargon, this is a robotic arm simulation coded in Love2d.

 I hope you like it if you manage to stumble around this repo, it was painful but at the same time fun learning all this stuff, and one day i plan to actually make this into a digital twin for my future real life 3d printed manipulator project that I plan to present at school. Have fun! ^-^

 **Controls:**
- `1`, `2`, `3`... -> Select a joint
- `Left / Right` -> Rotate selected joint
- `Up / Down` -> Extend / retract selected link
- `Space` -> Toggle auto (IK) mode (The arm follows mouse :D)
- Mouse movement -> Set target position

![Demo of simulation](assets/demo.gif)
