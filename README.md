# Robotic-Fruit-Sorter
Robotic Fruit Sorter is a computer vision and robotics simulation developed during the Stanford AI4ALL Summer Program 2026 by the Robotics cohort. The project was created to demonstrate how artificial intelligence and robotic automation can help address real-world agricultural challenges, specifically the time-consuming and expensive process of manually sorting fruits and vegetables. Inspired by United Nations Sustainable Development Goal 9 (Industry, Innovation and Infrastructure), the system showcases an accessible solution that combines computer vision, robotics, and motion planning into a single automated workflow.

The simulation begins by capturing an image of the workspace through a virtual camera. The image is processed using OpenCV, where it is converted into the HSV color space and analyzed using thresholding and masking techniques. Thresholding separates objects from the background based on pixel intensity, while masking isolates the desired regions of the image. After preprocessing, contour detection identifies the boundaries of each fruit, allowing the system to calculate its position and determine where the robotic arm should move.

The robotic arm is built as a simple yet robust mechanism with two degrees of freedom, consisting of two links, two joints, and a gripper capable of securely grasping fruits. To reach a target, the simulation uses Inverse Kinematics, which computes the joint angles required for the end effector to move to a specified Cartesian coordinate. The robot then follows a smooth trajectory using Cartesian path planning, ensuring accurate and efficient motion while transporting each fruit to its corresponding basket.

The project is implemented primarily in Python using three main libraries. PyBullet provides the physics simulation, robotic arm control, camera system, and grasping mechanics. OpenCV performs image processing, including HSV thresholding and contour detection for fruit recognition. NumPy supports numerical operations, coordinate transformations, and camera matrix calculations throughout the simulation.

Overall, Robotic Fruit Sorter demonstrates how the integration of computer vision, robotics, and intelligent motion planning can automate repetitive agricultural tasks. Although implemented as a simulation, the project illustrates concepts widely used in industrial automation and highlights the potential of AI-powered robotic systems to improve efficiency, reduce labor requirements, and support more sustainable agricultural practices.

## Video of presentation
THE FOLLOWING LINK IS A DEMO:
[Video of the Presentation](./RobotSimulation.mp4)

The specific robotic arm was the final project for that Stanford AI4ALL Summer Program 2026 and specificely the Robotics cohort. We hope you liked it!

## Running the project
- Go to `Code`
- Install Dependencies with `uv sync` or `pip install -r requirements.txt`
- Run the `robotic_fruit_sorter_final.py`
