# Audio Beat Detector

The Audio Beat Detector block performs real-time beat and rhythm detection from
an incoming microphone stream. It is intended for music-reactive robots, LED
effects, kinetic installations, and event-driven audio interaction.

## Typical Use

Use this block when you need low-latency beat information to trigger lighting,
motion accents, or rhythm-aware behaviors on-device.

## Key Capabilities

* Detect percussive beats from live audio
* Publish rhythm events for downstream ROS2 nodes
* Drive WS2812 effects, movement cues, or audio-reactive automation

## Implementation

The current implementation lives in:

* `src/xpi_audio/xpi_audio/beat_detector_node.py`

## Related Blocks

* `blocks/sensors/audio/analyzer/README.md`
* `blocks/sensors/audio/respeaker/README.md`
* `blocks/actuators/leds/ws2812_effects/README.md`
