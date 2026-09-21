import unittest

from d435_python_camera.camera_info import calibration_from_values


class CameraInfoTest(unittest.TestCase):
    def test_maps_realsense_intrinsics(self):
        calibration = calibration_from_values(
            width=640,
            height=480,
            fx=615.0,
            fy=616.0,
            ppx=319.5,
            ppy=239.5,
            coeffs=[0.1, -0.2, 0.003, 0.004, 0.05],
        )

        self.assertEqual(calibration.width, 640)
        self.assertEqual(calibration.height, 480)
        self.assertEqual(calibration.distortion_model, 'plumb_bob')
        self.assertEqual(calibration.d, (0.1, -0.2, 0.003, 0.004, 0.05))
        self.assertEqual(
            calibration.k,
            (615.0, 0.0, 319.5, 0.0, 616.0, 239.5, 0.0, 0.0, 1.0),
        )
        self.assertEqual(
            calibration.r,
            (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(
            calibration.p,
            (
                615.0, 0.0, 319.5, 0.0,
                0.0, 616.0, 239.5, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ),
        )

    def test_rejects_invalid_dimensions(self):
        with self.assertRaisesRegex(ValueError, 'width and height must be positive'):
            calibration_from_values(
                width=0,
                height=480,
                fx=615.0,
                fy=616.0,
                ppx=319.5,
                ppy=239.5,
                coeffs=[0.0] * 5,
            )

    def test_rejects_invalid_focal_lengths(self):
        with self.assertRaisesRegex(ValueError, 'focal lengths must be positive'):
            calibration_from_values(
                width=640,
                height=480,
                fx=-1.0,
                fy=616.0,
                ppx=319.5,
                ppy=239.5,
                coeffs=[0.0] * 5,
            )

    def test_rejects_too_few_distortion_coefficients(self):
        with self.assertRaisesRegex(
            ValueError,
            'at least five distortion coefficients are required',
        ):
            calibration_from_values(
                width=640,
                height=480,
                fx=615.0,
                fy=616.0,
                ppx=319.5,
                ppy=239.5,
                coeffs=[0.0] * 4,
            )


if __name__ == '__main__':
    unittest.main()
