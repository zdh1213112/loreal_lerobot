#!/usr/bin/env python

# Copyright 2026 XenseRobotics Inc. All rights reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from unittest.mock import patch

import numpy as np

from lerobot.scripts import lerobot_record
from lerobot.utils.constants import ACTION, OBS_STR


class _FakePico4:
    name = "pico4"

    def __init__(self) -> None:
        self._enabled = False
        self.action_calls = 0
        self.reset_poses: list[np.ndarray] = []

    def get_action(self, current_tcp_pose_quat=None):
        self.action_calls += 1
        return {"tcp.x": 0.0, "gripper.pos": 0.0}

    def reset_to_pose(self, pose_7d, gripper_pos=0.0) -> None:
        self.reset_poses.append(np.asarray([*pose_7d, gripper_pos], dtype=np.float32))


class _FakeDobotNova5DH:
    name = "dobot_nova5_dh"
    robot_type = name
    action_features = {"tcp.x": float, "gripper.pos": float}

    def __init__(self) -> None:
        self.state = np.asarray([0.0, 0.0], dtype=np.float32)
        self.reset_calls = 0
        self.sent_actions = []

    def get_observation(self):
        return {"tcp.x": float(self.state[0]), "gripper.pos": float(self.state[1])}

    def get_current_tcp_pose_quat(self):
        return np.asarray([self.state[0], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, self.state[1]])

    def reset_to_initial_position(self) -> None:
        self.reset_calls += 1
        self.state = np.asarray([1.0, 1.0], dtype=np.float32)

    def send_action(self, action):
        self.sent_actions.append(action)
        return action


class _FakeDataset:
    fps = 30
    features = {
        f"{OBS_STR}.state": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["tcp.x", "gripper.pos"],
        },
        ACTION: {
            "dtype": "float32",
            "shape": (2,),
            "names": ["tcp.x", "gripper.pos"],
        },
    }

    def __init__(self) -> None:
        self.frames = []
        self.episode_buffer = {
            "size": 1,
            f"{OBS_STR}.state": [np.asarray([0.25, 0.75], dtype=np.float32)],
            ACTION: [np.asarray([-1.0, -1.0], dtype=np.float32)],
        }

    def add_frame(self, frame) -> None:
        self.frames.append(frame)
        self.episode_buffer[f"{OBS_STR}.state"].append(frame[f"{OBS_STR}.state"])
        self.episode_buffer[ACTION].append(frame[ACTION])
        self.episode_buffer["size"] += 1


class DobotNova5DHRecordResetTest(unittest.TestCase):
    def test_single_arm_reset_records_shifted_transition(self) -> None:
        events = {
            "stop_recording": False,
            "rerecord_episode": False,
            "exit_early": False,
            "go_start": True,
        }
        robot = _FakeDobotNova5DH()
        teleop = _FakePico4()
        dataset = _FakeDataset()
        sleep_calls = 0

        def run_reset_synchronously(robot, teleop, set_done) -> None:
            robot.reset_to_initial_position()
            set_done()

        def finish_after_post_reset_boundary(**kwargs) -> None:
            nonlocal sleep_calls
            sleep_calls += 1
            if sleep_calls >= lerobot_record.DOBOT_RESET_BOUNDARY_ALIGN_FRAMES + 4:
                events["stop_recording"] = True

        def identity_observation(observation):
            return observation

        with (
            patch.object(lerobot_record, "Teleoperator", _FakePico4),
            patch.object(lerobot_record, "refresh_listener_events", lambda events: None),
            patch.object(lerobot_record, "_record_loop_sleep", finish_after_post_reset_boundary),
            patch.object(
                lerobot_record,
                "_start_reset_in_background",
                run_reset_synchronously,
            ),
        ):
            lerobot_record.dobot_nova5_dh_record_loop(
                robot=robot,
                events=events,
                fps=dataset.fps,
                teleop_action_processor=lambda inputs: inputs[0],
                robot_action_processor=lambda inputs: inputs[0],
                robot_observation_processor=identity_observation,
                dataset=dataset,
                teleop=teleop,
                control_time_s=1.0,
                single_task="reset test",
            )

        self.assertEqual(robot.reset_calls, 1)
        self.assertEqual(len(robot.sent_actions), 1)
        self.assertEqual(len(dataset.frames), 2)
        np.testing.assert_array_equal(
            dataset.episode_buffer[ACTION][0],
            dataset.episode_buffer[f"{OBS_STR}.state"][0],
        )
        np.testing.assert_array_equal(
            dataset.frames[0][f"{OBS_STR}.state"],
            np.asarray([0.0, 0.0], dtype=np.float32),
        )
        np.testing.assert_array_equal(
            dataset.frames[0][ACTION],
            np.asarray([1.0, 1.0], dtype=np.float32),
        )
        self.assertEqual(dataset.frames[0]["task"], "reset test")
        self.assertEqual(teleop.action_calls, 2)
        self.assertEqual(
            len(teleop.reset_poses),
            lerobot_record.DOBOT_RESET_BOUNDARY_ALIGN_FRAMES + 2,
        )
        self.assertEqual(
            sleep_calls,
            lerobot_record.DOBOT_RESET_BOUNDARY_ALIGN_FRAMES + 4,
        )


if __name__ == "__main__":
    unittest.main()
