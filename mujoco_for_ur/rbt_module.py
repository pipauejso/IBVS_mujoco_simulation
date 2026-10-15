"""Kinematics from the simulated robot at the camera optical centre."""

import mujoco
import numpy as np


class RobotController:
    def __init__(self, model, data, camera_name="end_effector_camera"):
        self.model, self.data = model, data
        self.camera_id = model.camera(camera_name).id
        self.body_id = int(model.cam_bodyid[self.camera_id])
        self.num_joints = model.nv
        self.joint_names = [model.joint(i).name for i in range(model.njnt)]
        self._scratch = mujoco.MjData(model)

    def _state(self, joint_angles):
        if joint_angles is None:
            return self.data
        self._scratch.qpos[:] = joint_angles
        mujoco.mj_forward(self.model, self._scratch)
        return self._scratch

    def get_forward_kinematics(self, joint_angles=None):
        """World pose of the camera in MuJoCo's OpenGL convention."""
        data = self._state(joint_angles)
        pose = np.eye(4)
        pose[:3, :3] = data.cam_xmat[self.camera_id].reshape(3, 3)
        pose[:3, 3] = data.cam_xpos[self.camera_id]
        return pose

    def get_jacobian(self, joint_angles=None):
        """Map qdot to [v, omega] in optical axes: right, down, forward.

        mj_jac includes the camera offset from its parent body. The SAME
        OpenGL-to-optical rotation applies to linear AND angular velocity.
        """
        data = self._state(joint_angles)
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros_like(jp)
        mujoco.mj_jac(self.model, data, jp, jr,
                      data.cam_xpos[self.camera_id], self.body_id)
        world_to_optical = (np.diag([1., -1., -1.]) @
                            data.cam_xmat[self.camera_id].reshape(3, 3).T)
        return np.vstack((world_to_optical @ jp, world_to_optical @ jr))

    def get_robot_info(self):
        return {"name": "UR5e (MuJoCo)", "num_joints": self.num_joints,
                "joint_names": self.joint_names}

    def print_robot_info(self):
        print(self.get_robot_info())
