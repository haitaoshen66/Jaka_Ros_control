# Release Notes - Version 1.1.0

## Overview / Highlights
In this release, the JAKA ROS 2 package has been significantly enhanced. We have expanded support to include all officially available JAKA 6-DOF cobot models and integrated Gazebo simulation functionality with MoveIt RViz control. Additionally, the documentation has been updated to guide users through the new features and provide clear instructions for installing and using the Gazebo simulation environment.

## New Features
- **Expanded Cobot Support:**  
  The current release now supports all officially available JAKA 6-DOF cobot models, including:  
  - JAKA A series  
  - JAKA C series  
  - JAKA Pro series  
  - JAKA S series  
  - JAKA Zu series  
  - JAKA MiniCobo  
  This ensures full compatibility with MoveIt 2 and the ROS 2 ecosystem for a variety of industrial applications.

- **Integrated Gazebo Simulation with MoveIt RViz Control:**  
  - **Gazebo Simulation Environment:**  
    Enjoy full dynamic simulation of your robot with realistic physics interactions and real-world dynamics. This environment is perfect for testing control algorithms and evaluating the robot's dynamic response in a simulated scenario.
  - This package is designed to work with **Gazebo Ignition Fortress**, the recommended simulation environment for **ROS 2 Humble**, providing stable and high-performance physics-based simulations.
  - Although the package has been tested on **Gazebo Classic**, it is not recommended due to potential instability and performance issues.
  - **Updated Documentation:**  
    The documentation now covers all key aspects related to Gazebo simulation:  
    - **Section 1.4.1:** Introduces the Gazebo Distributions Supported  
    - **Section 2.1.3:** About Gazebo Installation  
    - **Section 4.3:** Gazebo Simulation Tutorial: Real-Time Trajectory Execution in Gazebo

## Improvements
- **Updated Documentation for Controller Versions:**  
  We have added a new section in our documentation to clarify the supported controller firmware versions:
  - The minimal supported version is **1.7.1.46**, which provides the essential functionalities.
  - For improved stability and enhanced features, the **recommended controller version is 1.7.2.16**.
  Users are encouraged to upgrade to the recommended version to ensure optimal performance and seamless integration with ROS 2 and MoveIt 2.

---

For more details, please refer to our updated [JAKA ROS 2 Documentation](jaka_ros2_documentation.md).
