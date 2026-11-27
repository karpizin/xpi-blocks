# Robot Face Design Directions

This document captures the two current design directions for the robot face system.

- `Variant A` is the implementation target for the current codebase.
- `Variant B` remains a product and visual direction for future exploration.

## Shared Principles

- The face is `eyes-first`: the main emotional signal comes from eye shape, openness, spacing, asymmetry, and motion timing.
- The outer rim is a state signal, not decoration.
- The default neutral state has `no mouth`.
- `Mouth`, `nose`, and `chin` are intentionally absent from the base visual system.
- Any future mouth usage should be rare, reserved for high-amplitude emotions, and optional.

## Variant A

### Product Positioning

`Variant A` is the calmer and more premium direction. It is designed for:

- in-vehicle assistants
- embedded premium robotics
- low-noise environments where the assistant should feel present but not theatrical
- systems that must stay readable at a glance and acceptable over long sessions

In product terms, `Variant A` should feel:

- restrained
- trustworthy
- background-friendly
- emotionally intelligent without being needy

### Visual Language

- Circular or near-circular black face surface
- Tiny vertical capsule eyes
- Very subtle outer rim
- No mouth in neutral or standard states
- No literal brows; “brow energy” is expressed via eye tilt, eyelid asymmetry, and timing
- Minimal color palette, mostly cool white / pale cyan accents on black

### Motion Language

- Small, precise changes
- Slow breathing in idle
- Soft blink cadence
- Modest eye drift rather than broad sweeping motion
- Rim participates only as a secondary signal

### Emotion Table

| Emotion | Product Meaning | Eye Behavior | Rim Behavior |
|---|---|---|---|
| `NEUTRAL` | Calm presence, ready state | Tiny balanced capsules | Barely visible |
| `IDLE` | Alive but unobtrusive | Neutral eyes with micro-drift | Very soft pulse |
| `ALERT` | Attention requested or safety-relevant state | Taller, more open eyes | Slightly brighter, cooler contour |
| `THINKING` | Processing / searching / reasoning | Mild asymmetry, small lateral drift | Faint partial accent, still restrained |
| `FOCUSED` | High-confidence concentration | Narrower, more precise eyes | Tight, slightly sharper contour |
| `CURIOUS` | Interest without playfulness | One eye subtly more open / shifted | Tiny asymmetric emphasis |
| `HAPPY` | Gentle positive response | Slightly softened, brighter eyes | Minimal lift in brightness |
| `SURPRISED` | Unexpected event | Eyes open wider and lift | Brief crisp contour increase |
| `SAD` | Low-energy acknowledgement | Lower, heavier eyes | Dimmer, softer rim |
| `SLEEPY` | Rest / low-power / night tone | Shorter, droopier capsules | Rim fades toward near-off |
| `WINK` | High-signal special response | One eye compressed, one lively | Optional momentary accent |

### Strengths

- Strong premium feel
- High readability
- Low risk of “toy robot” aesthetics
- Easier to integrate into automotive or architectural hardware

### Risks

- Can feel too restrained if timing is weak
- Needs excellent motion tuning to avoid emotional flatness
- Some emotions require more nuance because the available graphic language is intentionally small

## Variant B

### Product Positioning

`Variant B` is the warmer and more expressive direction. It is designed for:

- companion robots
- home assistants with stronger social presence
- products where the assistant should feel emotionally legible from farther away
- interaction models where “attention” and “acknowledgement” are central

In product terms, `Variant B` should feel:

- warm
- attentive
- gently charismatic
- more obviously alive

### Visual Language

- Softly rounded premium display face
- Small glowing oval eyes
- More active outer rim or halo
- No mouth by default
- More abstract than a literal face, but more expressive than `Variant A`
- Slightly more visible atmospheric glow

### Motion Language

- Eyes remain primary, but rim participates actively
- The rim can carry attention, cognition, wakefulness, and warmth
- More asymmetry is acceptable
- Slightly broader state transitions than `Variant A`

### Emotion Table

| Emotion | Product Meaning | Eye Behavior | Rim Behavior |
|---|---|---|---|
| `NEUTRAL` | Calm warm standby | Soft ovals, balanced glow | Full low-intensity halo |
| `LISTENING` | Active attention to user input | Slight narrowing / centering | Gentle breathing glow |
| `ACKNOWLEDGE` | “I heard you” | Brief soft lift / brighten | Short confirmation pulse |
| `THINKING` | Visible cognition | Subtle asymmetry and drift | Partial moving arc or offset halo |
| `CURIOUS` | Social interest | One eye shifts or tilts more clearly | One-sided emphasis |
| `DELIGHT` | Positive warmth | Softer, slightly lifted eyes | Warmer brighter halo |
| `ALERT` | Important state | Sharper, more awake eyes | Tighter brighter contour |
| `SLEEPY` | Wind-down or low-power tone | Droopier eyes, reduced glow | Halo fades and fragments softly |
| `AMUSED` | Light playfulness | Eyes soften into a friendlier shape | Warmer side emphasis |
| `SURPRISED` | Distinct event detection | Eyes widen noticeably | Brief bright ring expansion |

### Strengths

- Higher emotional legibility
- The rim provides a richer second channel without adding a mouth
- Better suited to overtly social companion behavior

### Risks

- Easier to over-animate
- Can drift toward “smart home gadget glow” if not tightly art-directed
- Slightly higher risk of losing the calm premium tone

## Current Implementation Decision

The repository should implement `Variant A` first.

Reasons:

- It matches the desired premium, laconic baseline better.
- It gives us a stricter visual grammar to refine in code.
- It keeps future expansion open: `Variant B` can later be implemented as an alternate renderer or style profile.

## Implementation Notes

For the current code pass:

- Keep the face mouthless
- Use eye shape and rim intensity as the main channels
- Make the rim passive and secondary
- Preserve the existing emotion names and topic API where possible
- Treat any mouth-related parameters as legacy compatibility fields, not primary rendering features
