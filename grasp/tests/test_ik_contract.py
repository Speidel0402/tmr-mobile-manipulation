import unittest

from rim_grasp_perception.ik_contract import (
    LEFT_JOINT_NAMES,
    moveit_error_name,
    normalized_quaternion_xyzw,
    ordered_joint_values,
)


class IkContractTests(unittest.TestCase):
    def test_joint_seed_is_reordered_by_name(self):
        names = list(reversed(LEFT_JOINT_NAMES))
        values = list(range(7))
        self.assertEqual(
            ordered_joint_values(names, values), list(reversed(values))
        )

    def test_incomplete_seed_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing joints"):
            ordered_joint_values(LEFT_JOINT_NAMES[:-1], [0.0] * 6)

    def test_quaternion_is_normalized_and_zero_rejected(self):
        self.assertEqual(
            normalized_quaternion_xyzw([0, 0, 0, 2]), [0.0, 0.0, 0.0, 1.0]
        )
        with self.assertRaisesRegex(ValueError, "zero quaternion"):
            normalized_quaternion_xyzw([0, 0, 0, 0])

    def test_moveit_error_is_human_readable(self):
        self.assertEqual(moveit_error_name(1), "SUCCESS")
        self.assertEqual(moveit_error_name(-21), "FRAME_TRANSFORM_FAILURE")
        self.assertEqual(moveit_error_name(-31), "NO_IK_SOLUTION")


if __name__ == "__main__":
    unittest.main()
