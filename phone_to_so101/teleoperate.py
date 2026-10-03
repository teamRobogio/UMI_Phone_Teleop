# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specif

import time

import numpy as np

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.phone import Phone, PhoneConfig
from lerobot.teleoperators.phone.config_phone import PhoneOS
from lerobot.teleoperators.phone.phone_processor import MapPhoneActionToRobotAction
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.rotation import Rotation
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30

# Hold the phone top-edge forward / charging port toward you. LeRobot's mapping expects the
# phone turned 180 deg about the screen normal, so rotate the phone's motion back by 180 deg.
PHONE_TOP_FORWARD = True
_FLIP_Z180 = Rotation.from_rotvec(np.array([0.0, 0.0, np.pi]))

# Remap phone rotations to gripper rotations:
#   phone nose down/up (about short axis)  -> wrist up/down
#   phone roll left/right (about long axis) -> gripper twist (wrist roll)
#   phone flat turn (about screen normal)   -> sideways gripper rotation
# Signs: +1 or -1 to reverse a direction, 0 to disable that rotation.
# Tap B1 once to start following the phone, tap again to stop and hold (instead of holding B1).
B1_TOGGLE = True

REMAP_ROTATION_AXES = True
PITCH_SIGN = 1
ROLL_SIGN = 1
YAW_SIGN = 1


def main():
    # Initialize the robot and teleoperator
    robot_config = SO101FollowerConfig(
        port="COM14", id="so-arm101", use_degrees=True
    )
    teleop_config = PhoneConfig(phone_os=PhoneOS.IOS)  # or PhoneOS.ANDROID

    # Initialize the robot and teleoperator
    robot = SO101Follower(robot_config)
    teleop_device = Phone(teleop_config)

    # NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
    kinematics_solver = RobotKinematics(
        urdf_path="C:/Users/giorg/umi-trajectory/third_party/SO-ARM100/Simulation/SO101/so101_new_calib.urdf",
        target_frame_name="gripper_frame_link",
        joint_names=list(robot.bus.motors.keys()),
    )

    # Build pipeline to convert phone action to ee pose action to joint action
    phone_to_robot_joints_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=[
            MapPhoneActionToRobotAction(platform=teleop_config.phone_os),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                motor_names=list(robot.bus.motors.keys()),
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.10,
            ),
            GripperVelocityToJoint(
                speed_factor=20.0,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # Connect to the robot and teleoperator
    robot.connect()
    teleop_device.connect()

    # Init rerun viewer
    init_rerun(session_name="phone_so100_teleop")

    if not robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    print("Starting teleop loop. Move your phone to teleoperate the robot...")
    following = False
    b1_prev = True  # B1 may still be held from calibration; require a release before the first toggle
    while True:
        t0 = time.perf_counter()

        # Get robot observation
        robot_obs = robot.get_observation()

        # Get teleop action
        phone_obs = teleop_device.get_action()
        if not phone_obs:  # no phone pose this cycle (dropped packet / ARKit lost tracking): hold
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
            continue
        if B1_TOGGLE:
            # The phone re-zeroes its position on every B1 press, so the press that turns
            # following on starts from zero motion, exactly like holding B1 does.
            b1 = bool(phone_obs["phone.raw_inputs"].get("b1", 0))
            if b1 and not b1_prev:
                following = not following
                print("FOLLOWING" if following else "HOLDING", flush=True)
            b1_prev = b1
            phone_obs["phone.enabled"] = following
        if PHONE_TOP_FORWARD:
            phone_obs["phone.pos"] = _FLIP_Z180.apply(phone_obs["phone.pos"])
            phone_obs["phone.rot"] = _FLIP_Z180 * phone_obs["phone.rot"] * _FLIP_Z180.inv()
        if REMAP_ROTATION_AXES:
            # LeRobot's mapping drives: slot 1 -> wrist up/down, slot 2 -> twist, slot 0 -> sideways
            rv = phone_obs["phone.rot"].as_rotvec()  # [about short axis, about long axis, about screen normal]
            phone_obs["phone.rot"] = Rotation.from_rotvec(
                np.array([YAW_SIGN * rv[2], PITCH_SIGN * rv[0], ROLL_SIGN * rv[1]])
            )

        # Phone -> EE pose -> Joints transition
        joint_action = phone_to_robot_joints_processor((phone_obs, robot_obs))

        # Send action to robot
        _ = robot.send_action(joint_action)

        # Visualize
        log_rerun_data(observation=phone_obs, action=joint_action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()
