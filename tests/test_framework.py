"""Framework guards, independent of unfinished student implementations."""
import unittest
from unittest.mock import patch

import numpy as np

from test_reproduction import ServoFixture


class FrameworkIntegrationTests(ServoFixture, unittest.TestCase):
    def reference(self):
        return np.array([[100., 60.], [220., 60.], [220., 180.], [100., 180.]])

    def run_command(self, command):
        points = self.reference()
        self.servo.set_desired_features(points)
        with patch("visual_servo_module.detect_feature_points",
                   return_value=(points + [8., -5.], True)), \
             patch("visual_servo_module.compute_joint_velocity", return_value=command):
            return self.servo.process_visual_servo(
                self.robot, image=np.zeros((240, 320, 3), np.uint8))

    def test_velocity_limit_preserves_direction(self):
        raw = np.array([2., -1., .5, 0., .2, -.1])
        np.testing.assert_allclose(self.run_command(raw), raw / 4)

    def test_nonfinite_command_stops(self):
        np.testing.assert_array_equal(self.run_command(np.full(6, np.nan)), np.zeros(6))

    def test_wrong_command_shape_reports_interface_error(self):
        with self.assertRaisesRegex(ValueError, "B2"):
            self.run_command(np.ones((6, 1)))

    def test_wrong_detection_shape_reports_interface_error(self):
        with patch("visual_servo_module.detect_feature_points",
                   return_value=(np.zeros((3, 2)), True)):
            with self.assertRaisesRegex(ValueError, "A"):
                self.servo.get_feature_points(np.zeros((240, 320, 3), np.uint8))

    def test_converged_frame_does_not_call_student_controller(self):
        points = self.reference()
        self.servo.set_desired_features(points)
        with patch("visual_servo_module.detect_feature_points", return_value=(points, True)), \
             patch("visual_servo_module.compute_joint_velocity") as control:
            velocity = self.servo.process_visual_servo(
                self.robot, image=np.zeros((240, 320, 3), np.uint8))
        np.testing.assert_array_equal(velocity, np.zeros(6))
        control.assert_not_called()

    def test_task_exception_is_not_hidden_as_detection_loss(self):
        with patch("visual_servo_module.detect_feature_points",
                   side_effect=NotImplementedError("TODO A")):
            with self.assertRaisesRegex(NotImplementedError, "TODO A"):
                self.servo.get_feature_points(np.zeros((240, 320, 3), np.uint8))


if __name__ == "__main__":
    unittest.main()
