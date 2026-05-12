import numpy as np
import yaml
from PIL import Image, ImageDraw


LID_TOP_LEFT = 0
LID_BOTTOM_LEFT = 1
LID_TOP_RIGHT = 2
LID_BOTTOM_RIGHT = 3
PUPIL_SCALE_LEFT = 4
PUPIL_SCALE_RIGHT = 5
Y_OFFSET = 6
X_SHIFT = 7
BROW_TILT_LEFT = 8
BROW_TILT_RIGHT = 9
BROW_Y_OFFSET = 10
MOUTH_CURVE = 11
MOUTH_OPEN = 12

EXPRESSION_VECTOR_SIZE = 13
EXPRESSION_KEYS = (
    'lid_top_left',
    'lid_bottom_left',
    'lid_top_right',
    'lid_bottom_right',
    'pupil_scale_left',
    'pupil_scale_right',
    'y_offset',
    'x_shift',
    'brow_tilt_left',
    'brow_tilt_right',
    'brow_y_offset',
    'mouth_curve',
    'mouth_open',
)

DEFAULT_EXPRESSION_SPEC = {
    'lid_top_left': 5.0,
    'lid_bottom_left': 5.0,
    'lid_top_right': 5.0,
    'lid_bottom_right': 5.0,
    'pupil_scale_left': 1.0,
    'pupil_scale_right': 1.0,
    'y_offset': 0.0,
    'x_shift': 0.0,
    'brow_tilt_left': 0.0,
    'brow_tilt_right': 0.0,
    'brow_y_offset': 0.0,
    'mouth_curve': 0.0,
    'mouth_open': 0.0,
}


def normalize_expression_spec(spec):
    """Normalizes legacy or structured expression specs into a 13-value vector."""
    if isinstance(spec, np.ndarray):
        values = spec.tolist()
    else:
        values = spec

    if isinstance(values, dict):
        merged = DEFAULT_EXPRESSION_SPEC.copy()
        merged.update(values)
        unknown = set(merged.keys()) - set(EXPRESSION_KEYS)
        if unknown:
            raise ValueError(f'Unknown expression keys: {sorted(unknown)}')
        return np.array([float(merged[key]) for key in EXPRESSION_KEYS], dtype=float)

    if isinstance(values, (list, tuple)):
        if len(values) == 9:
            lid_t, lid_b, pupil_s, y_off, x_shift, brow_t, brow_y, mouth_c, mouth_o = values
            return np.array([
                float(lid_t),
                float(lid_b),
                float(lid_t),
                float(lid_b),
                float(pupil_s),
                float(pupil_s),
                float(y_off),
                float(x_shift),
                float(brow_t),
                float(brow_t),
                float(brow_y),
                float(mouth_c),
                float(mouth_o),
            ], dtype=float)

        if len(values) == EXPRESSION_VECTOR_SIZE:
            return np.array([float(v) for v in values], dtype=float)

    raise ValueError('Expression spec must be a dict or a list with 9 or 13 values.')


def load_emotions_from_yaml(yaml_path):
    with open(yaml_path, 'r') as handle:
        raw_data = yaml.safe_load(handle) or {}

    normalized = {}
    for name, spec in raw_data.items():
        normalized[name.upper()] = normalize_expression_spec(spec)

    if 'NEUTRAL' not in normalized:
        normalized['NEUTRAL'] = normalize_expression_spec(DEFAULT_EXPRESSION_SPEC)

    if 'IDLE_ANIMATED' not in normalized:
        normalized['IDLE_ANIMATED'] = normalized['NEUTRAL'].copy()

    return normalized


def render_face_image(params, canvas_size=(256, 256)):
    """Renders a face to a PIL image from a normalized parameter vector."""
    p = normalize_expression_spec(params)
    img = Image.new('RGB', canvas_size, color=(0, 0, 0))
    draw = ImageDraw.Draw(img)

    eye_w, eye_h = 60, 80
    eye_spacing = 100
    center_x, center_y = canvas_size[0] // 2, canvas_size[1] // 2

    eyes = [
        {
            'side': -1,
            'lid_top': p[LID_TOP_LEFT],
            'lid_bottom': p[LID_BOTTOM_LEFT],
            'pupil_scale': p[PUPIL_SCALE_LEFT],
            'brow_tilt': p[BROW_TILT_LEFT],
        },
        {
            'side': 1,
            'lid_top': p[LID_TOP_RIGHT],
            'lid_bottom': p[LID_BOTTOM_RIGHT],
            'pupil_scale': p[PUPIL_SCALE_RIGHT],
            'brow_tilt': p[BROW_TILT_RIGHT],
        },
    ]

    for eye in eyes:
        cx = center_x + (eye['side'] * eye_spacing // 2) + p[X_SHIFT]
        cy = center_y + p[Y_OFFSET] - 20
        radius = int(eye_w * eye['pupil_scale'] / 2)
        draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill=(255, 255, 255))

        draw.chord(
            [cx - eye_w, cy - eye_h - 10, cx + eye_w, cy + eye_h - 10],
            start=180 - eye['lid_top'],
            end=360 + eye['lid_top'],
            fill=(0, 0, 0),
        )
        draw.chord(
            [cx - eye_w, cy - eye_h + 10, cx + eye_w, cy + eye_h + 10],
            start=0 - eye['lid_bottom'],
            end=180 + eye['lid_bottom'],
            fill=(0, 0, 0),
        )

        brow_len = 50
        brow_center_y = center_y + p[Y_OFFSET] - 70 + p[BROW_Y_OFFSET]
        x1 = cx - brow_len // 2
        y1 = brow_center_y - eye['brow_tilt'] / 2
        x2 = cx + brow_len // 2
        y2 = brow_center_y + eye['brow_tilt'] / 2
        draw.line([x1, y1, x2, y2], fill=(255, 255, 255), width=8)

    mouth_x = center_x + p[X_SHIFT] * 0.5
    mouth_y = center_y + 60 + p[Y_OFFSET] * 0.5
    mouth_width = 80
    if p[MOUTH_OPEN] > 5:
        draw.ellipse(
            [
                mouth_x - mouth_width // 2,
                mouth_y - p[MOUTH_OPEN] / 2,
                mouth_x + mouth_width // 2,
                mouth_y + p[MOUTH_OPEN] / 2,
            ],
            fill=(255, 255, 255),
        )
    else:
        bbox = [
            mouth_x - mouth_width // 2,
            mouth_y - abs(p[MOUTH_CURVE]) - 10,
            mouth_x + mouth_width // 2,
            mouth_y + abs(p[MOUTH_CURVE]) + 10,
        ]
        if p[MOUTH_CURVE] >= 0:
            draw.arc(bbox, start=0, end=180, fill=(255, 255, 255), width=6)
        else:
            draw.arc(bbox, start=180, end=360, fill=(255, 255, 255), width=6)

    return img
