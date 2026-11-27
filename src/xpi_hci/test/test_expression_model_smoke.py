import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from xpi_hci.expression_model import (
    DEFAULT_EXPRESSION_SPEC,
    EXPRESSION_VECTOR_SIZE,
    LID_BOTTOM_LEFT,
    LID_BOTTOM_RIGHT,
    LID_TOP_LEFT,
    LID_TOP_RIGHT,
    apply_animation_overlay,
    load_emotions_from_yaml,
    normalize_expression_spec,
    render_face_image,
)


class TestExpressionModelSmoke(unittest.TestCase):
    def setUp(self):
        self.yaml_path = Path(__file__).resolve().parents[1] / 'config' / 'emotions.yaml'

    def test_legacy_expression_vector_is_normalized(self):
        vector = normalize_expression_spec([5, 10, 1.0, 0, 0, 0, 0, 0, 0])
        self.assertEqual(len(vector), EXPRESSION_VECTOR_SIZE)
        self.assertEqual(vector[LID_TOP_LEFT], vector[LID_TOP_RIGHT])
        self.assertEqual(vector[LID_BOTTOM_LEFT], vector[LID_BOTTOM_RIGHT])

    def test_asymmetric_wink_is_loaded_from_yaml(self):
        emotions = load_emotions_from_yaml(self.yaml_path)
        self.assertIn('WINK', emotions)
        wink = emotions['WINK']
        self.assertGreater(wink[LID_TOP_LEFT], wink[LID_TOP_RIGHT])
        self.assertGreater(wink[LID_BOTTOM_LEFT], wink[LID_BOTTOM_RIGHT])

    def test_render_face_smoke(self):
        image = render_face_image(DEFAULT_EXPRESSION_SPEC, canvas_size=(128, 128))
        self.assertEqual(image.size, (128, 128))
        pixels = np.array(image)
        self.assertTrue(np.any(pixels > 0))

    def test_animation_overlay_keeps_expression_shape(self):
        animated = apply_animation_overlay(DEFAULT_EXPRESSION_SPEC, 'IDLE_ANIMATED', t=1.5, blink_level=0.4)
        self.assertEqual(len(animated), EXPRESSION_VECTOR_SIZE)
        self.assertGreater(animated[LID_TOP_LEFT], DEFAULT_EXPRESSION_SPEC['lid_top_left'])
        self.assertGreater(animated[LID_TOP_RIGHT], DEFAULT_EXPRESSION_SPEC['lid_top_right'])

    def test_wink_render_is_visibly_asymmetric(self):
        emotions = load_emotions_from_yaml(self.yaml_path)
        image = render_face_image(emotions['WINK'], canvas_size=(160, 160))
        pixels = np.array(image).mean(axis=2)
        left_half = pixels[:, :80].sum()
        right_half = pixels[:, 80:].sum()
        self.assertNotAlmostEqual(left_half, right_half, delta=5000)

    def test_variant_a_neutral_keeps_lower_face_dark(self):
        emotions = load_emotions_from_yaml(self.yaml_path)
        image = render_face_image(emotions['NEUTRAL'], canvas_size=(160, 160))
        pixels = np.array(image).mean(axis=2)
        eye_band = pixels[48:88, 40:120].mean()
        lower_face = pixels[112:148, 56:104].mean()
        self.assertGreater(eye_band, lower_face * 2.0)


if __name__ == '__main__':
    unittest.main()
