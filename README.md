So, for my 2nd project, this is a 2D Love2D simulation of manipulator with a manual control and a mode with inverse kinematics that is implemented using the cyclic coordinate descent (or CCD) algorithm. The config dynamicly adapts for any amount of segments specified, their lenghts (oh and also it scales the step size by the lenghts) and starting angles. It even has relative angle clamps!1!1! And uhm theres various protections for when the angle exceeds 2pi or when the angles become zero ah yeah also theres a visual marking for when the target is unreachable and a cool graphic vizualization of the arm (it even has a shadow). Im not very good at writing this stuff but i hope this explains it.

 If we remove the scary technical jargon, this is a robotic arm simulation coded in Love2d.

 I hope you like it if you manage to stumble around this repo, it was painful but at the same time fun learning all this stuff, and one day i plan to actually make this into a digital twin for my future real life 3d printed manipulator project that I plan to present at school. Have fun! ^-^

 **Controls:**
- `1`, `2`, `3`... -> Select a joint
- `Left / Right` -> Rotate selected joint
- `Up / Down` -> Extend / retract selected link
- `Space` -> Toggle auto (IK) mode (The arm follows mouse :D)
- Mouse movement -> Set target position


***DEMO OF THE SIMULATION***

![Demo of the simulation](https://raw.githubusercontent.com/N3G3NTR0PY/Love2d-CCD-IK-Solver/refs/heads/main/assets/demo.gif)
