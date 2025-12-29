# Hexapod Master Implementation Plan

The project is divided into 5 stages, each adding a new level of intelligent behavior.

## Stage 1: Geometry and IK of a Single Leg (The Leg)
*   [x] Mathematical IK calculation for a 3-link leg (Coxa, Femur, Tibia). See `docs/LEG_IK_MATH.md`.
*   [x] Creation of a URDF model for a single leg for visualization. See `urdf/leg.urdf`.
*   [x] Implementation of the interactive node `leg_ik_viewer_node.py`, linking the IK class with real-time visualization.
*   [x] Testing in Rviz2: moving the leg to given (x, y, z) coordinates.

## Stage 2: Body Kinematics (The Body)
*   [x] Implementation of the `BodyKinematics` class for calculating transformations of all 6 legs. See `scripts/body_kinematics.py`.
*   [x] Creation of the geometry configuration file `config/hexapod_config.yaml`.
*   [x] Creation of the integration node `hexapod_body_kinematics_node.py`.
*   [x] Full hexapod URDF model (6 legs on the body). See `urdf/hexapod.urdf`.
*   [x] Implementation of smooth "squatting" and tilting via the `/hexapod/body_pose` topic.

## Stage 3: Physical Simulation (The Void)
*   [x] URDF/Xacro configuration for Gazebo (adding mass, inertia, and collisions for all links). See `urdf/hexapod.urdf.xacro`.
*   [x] `ros2_control` configuration for managing 18 joints. See `config/hexapod_controllers.yaml`.
*   [x] Creation of a Launch file to start Gazebo and "spawn" the robot. See `scripts/gazebo.launch.py`.
*   [ ] Testing stability in the simulator with various body poses.

## Stage 4: Gait Generator (The Gait)
*   [x] Implementation of the `GaitEngine` with support for Tripod, Wave, and Ripple gaits.
*   [x] Integration of the gait engine into the ROS2 control loop.

## Stage 5: Manipulation and VLM (The Action)
*   [ ] Integration of the manipulator arm IK.
*   [ ] Coordination of the center of mass when the manipulator is extended (prevention of tipping).
*   [ ] Integration with VLM for object grasping based on visual coordinates.