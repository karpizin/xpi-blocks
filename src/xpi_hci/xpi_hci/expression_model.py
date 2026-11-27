import math

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFilter


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


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


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


def apply_animation_overlay(params, emotion_name, t, blink_level=0.0):
    """Applies a calm, eyes-first animation overlay for Variant A."""
    p = normalize_expression_spec(params).copy()
    emotion = emotion_name.upper()

    if blink_level > 0.0:
        blink_target = 88.0
        for index in (LID_TOP_LEFT, LID_BOTTOM_LEFT, LID_TOP_RIGHT, LID_BOTTOM_RIGHT):
            p[index] = p[index] + (blink_target - p[index]) * blink_level

    if emotion in {'NEUTRAL', 'IDLE', 'IDLE_ANIMATED'} or 'IDLE' in emotion:
        drift = math.sin(t * 0.28) * 2.0 + math.sin(t * 0.14 + 0.8) * 0.9
        breath = math.sin(t * 0.75) * 1.2
        pulse = 1.0 + math.sin(t * 0.55) * 0.012
        lid_rest = (math.sin(t * 0.38 + 1.1) + 1.0) * 0.55
        p[X_SHIFT] += drift
        p[Y_OFFSET] += breath
        p[LID_TOP_LEFT] += lid_rest
        p[LID_TOP_RIGHT] += lid_rest
        p[PUPIL_SCALE_LEFT] *= pulse
        p[PUPIL_SCALE_RIGHT] *= pulse

    elif emotion in {'HAPPY', 'JOYFUL', 'RELAXED', 'PROUD'}:
        soften = (math.sin(t * 0.9) + 1.0) * 0.8
        p[Y_OFFSET] -= abs(math.sin(t * 0.7)) * 1.0
        p[LID_TOP_LEFT] += soften
        p[LID_TOP_RIGHT] += soften
        p[PUPIL_SCALE_LEFT] *= 1.01
        p[PUPIL_SCALE_RIGHT] *= 1.01

    elif emotion in {'THINKING', 'CURIOUS', 'FOCUSED'}:
        gaze = math.sin(t * 0.75) * 9.0
        p[X_SHIFT] += gaze
        p[Y_OFFSET] -= abs(math.sin(t * 0.28)) * 0.8
        p[BROW_TILT_LEFT] -= math.sin(t * 0.48) * 1.6
        p[BROW_TILT_RIGHT] += math.sin(t * 0.48) * 1.1

    elif emotion in {'LAUGHING', 'ECSTATIC', 'EXCITED'}:
        delight = (math.sin(t * 1.9) + 1.0) * 0.5
        p[Y_OFFSET] -= delight * 1.7
        p[LID_TOP_LEFT] += delight * 1.2
        p[LID_TOP_RIGHT] += delight * 1.2
        p[PUPIL_SCALE_LEFT] *= 1.0 + delight * 0.025
        p[PUPIL_SCALE_RIGHT] *= 1.0 + delight * 0.025

    elif emotion in {'SURPRISED', 'SHOCKED', 'ALERT'}:
        settle = abs(math.sin(t * 1.1)) * 1.2
        p[Y_OFFSET] -= settle * 0.8
        p[PUPIL_SCALE_LEFT] *= 1.0 + settle * 0.01
        p[PUPIL_SCALE_RIGHT] *= 1.0 + settle * 0.01
        p[LID_TOP_LEFT] -= settle * 0.6
        p[LID_TOP_RIGHT] -= settle * 0.6

    elif emotion in {'SLEEPY', 'TIRED', 'DEPRESSED', 'BORED'}:
        droop = (math.sin(t * 0.42) + 1.0) * 1.35
        p[Y_OFFSET] += droop
        p[LID_TOP_LEFT] += droop * 0.8
        p[LID_TOP_RIGHT] += droop * 0.8
        p[PUPIL_SCALE_LEFT] *= 0.995
        p[PUPIL_SCALE_RIGHT] *= 0.995

    elif emotion in {'ANGRY', 'FURIOUS', 'EVIL'}:
        tension = (math.sin(t * 0.8) + 1.0) * 0.35
        p[LID_BOTTOM_LEFT] += 0.5 + tension
        p[LID_BOTTOM_RIGHT] += 0.5 + tension
        p[BROW_TILT_LEFT] += 1.5
        p[BROW_TILT_RIGHT] += 1.5

    elif emotion in {'SAD'}:
        p[Y_OFFSET] += abs(math.sin(t * 0.3)) * 0.7
        p[BROW_TILT_LEFT] -= 0.8
        p[BROW_TILT_RIGHT] += 0.8

    elif emotion in {'WINK', 'WINK_RIGHT'}:
        bright_pulse = 1.0 + math.sin(t * 2.6) * 0.02
        sway = math.sin(t * 1.4) * 2.0
        p[Y_OFFSET] -= abs(math.sin(t * 0.75)) * 0.6
        if emotion == 'WINK_RIGHT':
            p[PUPIL_SCALE_LEFT] *= bright_pulse
            p[X_SHIFT] += sway
        else:
            p[PUPIL_SCALE_RIGHT] *= bright_pulse
            p[X_SHIFT] -= sway

    return p


def render_face_image(params, canvas_size=(256, 256)):
    """Renders the Variant A face: tiny capsule eyes with a passive rim."""
    p = normalize_expression_spec(params)
    supersample = 4
    render_size = (canvas_size[0] * supersample, canvas_size[1] * supersample)
    img = Image.new('RGBA', render_size, color=(0, 0, 0, 255))

    center_x = render_size[0] // 2
    center_y = render_size[1] // 2
    face_radius = 106 * supersample
    eye_width = 18 * supersample
    eye_height = 62 * supersample
    eye_spacing = 92 * supersample

    def add_glow_ellipse(bbox, color, blur_radius, alpha_scale=1.0, outline_width=0):
        glow = Image.new('RGBA', render_size, (0, 0, 0, 0))
        glow_draw = ImageDraw.Draw(glow, 'RGBA')
        left, top, right, bottom = bbox
        layers = (
            (12 * supersample, 0.08),
            (7 * supersample, 0.12),
            (3 * supersample, 0.18),
        )
        for spread, opacity in layers:
            glow_draw.ellipse(
                [left - spread, top - spread // 2, right + spread, bottom + spread // 2],
                fill=(
                    color[0],
                    color[1],
                    color[2],
                    int(255 * opacity * alpha_scale),
                ),
            )
        if outline_width > 0:
            glow_draw.ellipse(
                bbox,
                outline=(color[0], color[1], color[2], int(255 * 0.22 * alpha_scale)),
                width=outline_width,
            )
        return glow.filter(ImageFilter.GaussianBlur(radius=blur_radius))

    def draw_eye_capsule(side, lid_top, lid_bottom, pupil_scale, brow_tilt):
        cx = center_x + (side * eye_spacing // 2) + int(p[X_SHIFT] * supersample * 0.55)
        cy = center_y - 2 * supersample + int(p[Y_OFFSET] * supersample * 0.95)

        lid_top_ratio = _clamp(lid_top / 90.0, -0.2, 1.15)
        lid_bottom_ratio = _clamp(lid_bottom / 90.0, -0.2, 1.15)
        openness = _clamp(1.06 - ((lid_top_ratio + lid_bottom_ratio) * 0.52), 0.12, 1.08)

        width = eye_width * _clamp(0.92 + (pupil_scale - 1.0) * 0.18, 0.86, 1.08)
        height = eye_height * openness * _clamp(0.96 + (pupil_scale - 1.0) * 0.08, 0.86, 1.08)
        tilt_deg = _clamp(-brow_tilt * 0.22, -8.0, 8.0)

        layer_size = int(max(width, height) * 2.4)
        eye_layer = Image.new('RGBA', (layer_size, layer_size), (0, 0, 0, 0))
        eye_draw = ImageDraw.Draw(eye_layer, 'RGBA')

        left = (layer_size - width) / 2
        top = (layer_size - height) / 2
        right = left + width
        bottom = top + height
        bbox = [left, top, right, bottom]

        if height < 12 * supersample:
            eye_draw.rounded_rectangle(
                [
                    left,
                    (layer_size / 2) - (3 * supersample),
                    right,
                    (layer_size / 2) + (3 * supersample),
                ],
                radius=3 * supersample,
                fill=(230, 244, 255, 214),
            )
        else:
            eye_draw.rounded_rectangle(
                bbox,
                radius=width / 2,
                fill=(240, 248, 255, 248),
            )
            inner_margin = max(2 * supersample, int(width * 0.18))
            eye_draw.rounded_rectangle(
                [
                    left + inner_margin,
                    top + inner_margin * 1.4,
                    right - inner_margin,
                    bottom - inner_margin * 1.4,
                ],
                radius=max(1, int((width - inner_margin * 2) / 2)),
                fill=(255, 255, 255, 88),
            )

        rotated = eye_layer.rotate(tilt_deg, resample=Image.Resampling.BICUBIC, expand=True)
        paste_x = int(cx - rotated.width / 2)
        paste_y = int(cy - rotated.height / 2)

        glow_bbox = [
            int(cx - width * 0.9),
            int(cy - height * 0.75),
            int(cx + width * 0.9),
            int(cy + height * 0.75),
        ]
        img.alpha_composite(
            add_glow_ellipse(
                glow_bbox,
                (100, 205, 255),
                blur_radius=6 * supersample,
                alpha_scale=_clamp(openness, 0.35, 1.0) * 0.75,
            )
        )
        img.alpha_composite(rotated, (paste_x, paste_y))

    avg_lids = (p[LID_TOP_LEFT] + p[LID_BOTTOM_LEFT] + p[LID_TOP_RIGHT] + p[LID_BOTTOM_RIGHT]) / 4.0
    avg_scale = (p[PUPIL_SCALE_LEFT] + p[PUPIL_SCALE_RIGHT]) / 2.0
    alertness = _clamp(1.12 - (avg_lids / 76.0) + ((avg_scale - 1.0) * 0.45), 0.18, 1.0)
    asymmetry = abs(p[X_SHIFT]) + abs(p[BROW_TILT_LEFT] - p[BROW_TILT_RIGHT]) + abs(p[BROW_Y_OFFSET])

    face_bbox = [
        center_x - face_radius,
        center_y - face_radius,
        center_x + face_radius,
        center_y + face_radius,
    ]
    img.alpha_composite(
        add_glow_ellipse(
            face_bbox,
            (88, 182, 230),
            blur_radius=9 * supersample,
            alpha_scale=0.28 + (alertness * 0.12),
            outline_width=max(1, supersample),
        )
    )

    rim_layer = Image.new('RGBA', render_size, (0, 0, 0, 0))
    rim_draw = ImageDraw.Draw(rim_layer, 'RGBA')
    rim_draw.ellipse(
        face_bbox,
        outline=(162, 222, 255, int(28 + (alertness * 26))),
        width=max(1, int(1.2 * supersample)),
    )
    img.alpha_composite(rim_layer.filter(ImageFilter.GaussianBlur(radius=1.2 * supersample)))

    if asymmetry > 5.0:
        accent_layer = Image.new('RGBA', render_size, (0, 0, 0, 0))
        accent_draw = ImageDraw.Draw(accent_layer, 'RGBA')
        accent_side = -1 if p[X_SHIFT] < 0 else 1
        if accent_side < 0:
            start, end = 208, 262
        else:
            start, end = -82, -28
        accent_draw.arc(
            [
                face_bbox[0] + 10 * supersample,
                face_bbox[1] + 10 * supersample,
                face_bbox[2] - 10 * supersample,
                face_bbox[3] - 10 * supersample,
            ],
            start=start,
            end=end,
            fill=(194, 239, 255, int(52 + min(asymmetry, 14) * 4)),
            width=max(1, int(1.4 * supersample)),
        )
        img.alpha_composite(accent_layer.filter(ImageFilter.GaussianBlur(radius=supersample)))

    draw_eye_capsule(-1, p[LID_TOP_LEFT], p[LID_BOTTOM_LEFT], p[PUPIL_SCALE_LEFT], p[BROW_TILT_LEFT])
    draw_eye_capsule(1, p[LID_TOP_RIGHT], p[LID_BOTTOM_RIGHT], p[PUPIL_SCALE_RIGHT], p[BROW_TILT_RIGHT])

    img = img.resize(canvas_size, Image.Resampling.LANCZOS)
    return img.convert('RGB')
