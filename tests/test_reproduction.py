"""Regression checks for geometry, correspondence, and stopping behavior."""
from pathlib import Path
import sys
import unittest

import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mujoco_for_ur"))
from main import load_simulation, optical_pose
from rbt_module import RobotController
from visual_servo_module import VisualServoController
from keyboard_controller import KeyboardController


class ReproductionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, _ = load_simulation()

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.robot = RobotController(self.model, self.data)
        self.servo = VisualServoController(self.model, self.data,
            "end_effector_camera", "target", (320, 240))

    def project(self, world_points):
        r, p = optical_pose(self.data, self.servo.camera_id)
        xyz = (world_points - p) @ r
        points = xyz[:, :2] / xyz[:, 2:3]
        return points * [self.servo.fx, self.servo.fy] + [self.servo.cx, self.servo.cy], xyz[:, 2]

    def test_full_image_jacobian_against_finite_differences(self):
        """Checks sign, all six axes, and the camera translational offset."""
        target = self.model.body("target").id
        centre = self.data.xpos[target].copy()
        r = self.data.xmat[target].reshape(3, 3).copy()
        world_points = np.array([[-.05, -.05, .01], [.05, -.05, .01],
                                 [.05, .05, .01], [-.05, .05, .01]]) @ r.T + centre
        qhome = self.data.qpos.copy()
        for offset in (np.zeros(6), np.array([.1, -.07, .05, .04, .08, -.12])):
            q = qhome + offset
            self.data.qpos[:] = q
            mujoco.mj_forward(self.model, self.data)
            points, depths = self.project(world_points)
            analytic = self.servo.calculate_interaction_matrix(points, depths) @ self.robot.get_jacobian()
            numeric = np.zeros((8, 6))
            eps = 1e-7
            for i in range(6):
                self.data.qpos[:] = q
                self.data.qpos[i] += eps
                mujoco.mj_forward(self.model, self.data)
                plus, _ = self.project(world_points)
                self.data.qpos[i] -= 2 * eps
                mujoco.mj_forward(self.model, self.data)
                minus, _ = self.project(world_points)
                numeric[:, i] = ((plus - minus) / [self.servo.fx, self.servo.fy]).ravel() / (2 * eps)
            np.testing.assert_allclose(analytic, numeric, atol=2e-7, rtol=2e-6)

    def test_perspective_corners_are_not_replaced_by_a_rectangle(self):
        image = np.zeros((240, 320, 3), np.uint8)
        expected = np.array([[90, 60], [220, 80], [200, 180], [110, 165]], np.float32)
        cv2.fillConvexPoly(image, expected.astype(np.int32), (255, 0, 0))
        actual, success = self.servo._extract_features_from_image(image)
        self.assertTrue(success)
        np.testing.assert_allclose(actual, expected, atol=1.5)

    def test_corner_identity_across_45_degree_boundary(self):
        base = np.array([[-45, -45], [45, -45], [45, 45], [-45, 45]])
        first = None
        for angle in range(35, 57, 3):
            r = Rotation.from_euler("z", angle, degrees=True).as_matrix()[:2, :2]
            polygon = base @ r.T + [160, 120]
            image = np.zeros((240, 320, 3), np.uint8)
            cv2.fillConvexPoly(image, np.int32(np.round(polygon)), (255, 0, 0))
            points, success = self.servo._extract_features_from_image(image)
            self.assertTrue(success)
            if first is None:
                first = points.copy()
            if self.servo.current_points is not None:
                self.assertLess(np.max(np.linalg.norm(points - self.servo.current_points, axis=1)), 7)
            self.servo.current_points = points

    def test_depth_is_paired_to_pixels_on_tilted_face(self):
        mocap = self.model.body_mocapid[self.model.body("target").id]
        original = Rotation.from_quat(self.data.mocap_quat[mocap][[1, 2, 3, 0]])
        self.data.mocap_quat[mocap] = (original * Rotation.from_euler(
            "xy", [12, -10], degrees=True)).as_quat()[[3, 0, 1, 2]]
        mujoco.mj_forward(self.model, self.data)
        geom = self.servo.geom_id
        local = np.array([[-.05, -.05, .01], [.05, -.05, .01],
                          [.05, .05, .01], [-.05, .05, .01]])
        world = local @ self.data.geom_xmat[geom].reshape(3, 3).T + self.data.geom_xpos[geom]
        points, expected = self.project(world)
        order = [2, 0, 3, 1]
        actual = self.servo._get_depth_for_points(points[order])
        np.testing.assert_allclose(actual, expected[order], atol=1e-12)

    def test_feature_loss_commands_zero_velocity(self):
        self.servo.set_desired_features(np.array([[100, 60], [220, 60], [220, 180], [100, 180]]))
        velocity = self.servo.process_visual_servo(self.robot,
                    image=np.zeros((240, 320, 3), np.uint8))
        np.testing.assert_array_equal(velocity, np.zeros(6))
        self.assertIsNone(self.servo.error_rms_px)

    def test_clipped_target_is_rejected(self):
        image = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(image, (0, 50), (80, 150), (255, 0, 0), -1)
        points, success = self.servo._extract_features_from_image(image)
        self.assertFalse(success)
        self.assertIsNone(points)

    def test_failed_reference_does_not_destroy_previous_reference(self):
        reference = np.array([[100, 60], [220, 60], [220, 180], [100, 180]])
        self.assertTrue(self.servo.set_desired_features(reference))
        reference[:] = 0  # stored reference must be a copy
        self.assertFalse(self.servo.set_desired_features(None))
        self.assertEqual(self.servo.desired_points[0, 0], 100)

    def test_windows_arrow_keys_move_target(self):
        keyboard = KeyboardController(self.model, self.data, step_size=.01)
        initial = self.data.mocap_pos[keyboard.mocap_id].copy()
        self.assertTrue(keyboard.process_key_input(2555904))
        np.testing.assert_allclose(self.data.mocap_pos[keyboard.mocap_id], initial + [.01, 0, 0])

    def test_invalid_depth_is_rejected(self):
        with self.assertRaises(ValueError):
            self.servo.calculate_interaction_matrix(np.zeros((4, 2)), [1, -1, 1, 1])

    def test_world_axis_rotation_at_initial_gimbal_lock_pose(self):
        keyboard = KeyboardController(self.model, self.data)
        before = self.data.mocap_quat[keyboard.mocap_id][[1, 2, 3, 0]]
        expected = (Rotation.from_rotvec([0, 0, .1]) *
                    Rotation.from_quat(before)).as_matrix()
        keyboard.process_key_input(ord('3'))
        after = self.data.mocap_quat[keyboard.mocap_id][[1, 2, 3, 0]]
        np.testing.assert_allclose(Rotation.from_quat(after).as_matrix(), expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
