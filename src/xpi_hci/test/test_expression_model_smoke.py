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


if __name__ == '__main__':
    unittest.main()
