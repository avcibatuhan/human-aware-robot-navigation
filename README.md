# Human-Aware Robot Navigation

🚧 **Work in progress.**

Vision-based human detection and tracking feeding a social costmap in ROS 2 Nav2.
A TurtleBot3 Waffle in Gazebo detects people with YOLOv8n in its RGB-D camera
stream, tracks them with DeepSORT, projects each track into the `map` frame, and
a custom Nav2 costmap layer adds a Gaussian-like cost around every tracked
person so the robot keeps its distance. Baseline and social runs are recorded
with rosbag2 and compared offline on human–robot clearance.

Stack: ROS 2 Jazzy · Gazebo Harmonic · Nav2 · YOLOv8 · DeepSORT.

Setup instructions, the architecture diagram and evaluation results will be
added as each part is built and measured.

## License

[AGPL-3.0](LICENSE)
