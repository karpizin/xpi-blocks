# 20 CV & VLM Project Ideas for Robotics

This document outlines a roadmap of Computer Vision (CV) and Vision-Language Model (VLM) scenarios, ranging from educational "Hello World" projects to advanced, real-world applications. These ideas drive the development of `xpi-blocks`.

## Category 1: Educational (The "Hello World" of AI Robotics)
*Simple projects to understand core principles using standard libraries (OpenCV) and minimal resources.*

1.  **"Follow the Red Ball" (Classic CV)**
    *   **Goal:** Robot rotates and drives towards a specific colored object.
    *   **Tech:** OpenCV (HSV color filtering, contour detection), PID controller.
    *   **Hardware:** Camera, 2WD/4WD chassis.

2.  **"Don't Fall Off" (Visual Cliff Detection)**
    *   **Goal:** Detect table edges visually to prevent falls, replacing IR sensors.
    *   **Tech:** OpenCV (Canny edge detection, ROI filtering).
    *   **Hardware:** Downward-facing camera.

3.  **"Gesture Control"**
    *   **Goal:** Control robot movement with hand gestures (Stop, Go, Left, Right).
    *   **Tech:** MediaPipe Hands (lightweight pose estimation).
    *   **Hardware:** Camera.

4.  **"Face Keeper" (Pan/Tilt)**
    *   **Goal:** A robotic head that keeps a human face centered in the frame.
    *   **Tech:** Haar Cascades or MediaPipe Face Detection + Servos.
    *   **Hardware:** Camera on Pan/Tilt servo mount.

5.  **"Robust Line Follower"**
    *   **Goal:** Follow a line on the floor visually, handling gaps or dotted lines better than IR.
    *   **Tech:** OpenCV (Thresholding, perspective transform/Bird's Eye View).
    *   **Hardware:** Camera.

6.  **"Emotion Mirror"**
    *   **Goal:** Robot displays an emoji on its screen matching the user's facial expression.
    *   **Tech:** DeepFace or MediaPipe + LED Matrix/OLED.
    *   **Hardware:** Camera, Display.

7.  **"Visual Odometry Lite"**
    *   **Goal:** Estimate relative movement (flow) from video to approximate distance traveled.
    *   **Tech:** OpenCV (Optical Flow, Lucas-Kanade).
    *   **Hardware:** Camera.

## Category 2: VLM & Reasoning (Intelligent Robotics)
*Using LLMs/VLMs (Gemini, GPT-4o, Llava) to understand the scene semantically.*

8.  **"VLM Observer" (Implemented)**
    *   **Goal:** Robot periodically looks at the world and describes what it sees in a text topic.
    *   **Tech:** `vlm_observer_node`, Gemini/OpenRouter API.
    *   **Status:** ✅ Basic implementation ready.

9.  **"Find the Keys" (Semantic Search)**
    *   **Goal:** Command: "Find my keys." Robot spins/explores until VLM confirms keys are in the frame.
    *   **Tech:** VLM with specific prompt ("Is object X in this image? Return YES/NO").

10. **"Safety Inspector"**
    *   **Goal:** Robot patrols and alerts on hazards (spilled water, open fire, person lying down).
    *   **Tech:** VLM prompted for safety assessment.

11. **"Semantic Sorting"**
    *   **Goal:** A robotic arm sorts items based on vague criteria ("Put electronics here, organic trash there").
    *   **Tech:** VLM + Manipulator.

12. **"Visual Q&A Agent"**
    *   **Goal:** Remote avatar. Operator asks via chat: "Where is the cat?", robot replies with photo + VLM analysis.
    *   **Tech:** Chat interface (Telegram/Web) bridged to ROS2.

13. **"Contextual Navigation"**
    *   **Goal:** "Go to the sofa." Robot identifies "sofa" in video stream and navigates towards it.
    *   **Tech:** Object detection (YOLO or VLM) + Visual Servoing.

## Category 3: Applied / Real-World
*Practical utilities for home or industry.*

14. **"Gardener Bot"**
    *   **Goal:** Monitor plant health. Detect wilting leaves or pests.
    *   **Tech:** VLM or specialized CV model (classification).

15. **"Pet Photographer"**
    *   **Goal:** Robot follows a pet and automatically snaps photos when the pet does something "cute" or looks at the camera.
    *   **Tech:** Pet detection + VLM "cuteness" scoring.

16. **"Smart Security Scout"**
    *   **Goal:** Patrol mode. If an unknown person is seen, send alert with description.
    *   **Tech:** Face Recognition + VLM for description ("Male, wearing red hoodie").

17. **"Inventory Counter"**
    *   **Goal:** Drive past a shelf and count specific items (cans, boxes).
    *   **Tech:** Object counting (YOLO or specialized counting models).

18. **"Reading Assistant"**
    *   **Goal:** Robot for visually impaired. User points camera at text/sign, robot reads it via TTS.
    *   **Tech:** OCR (Tesseract/Google Vision) + TTS.

19. **"Precision Docking"**
    *   **Goal:** Dock to charging station using a visual marker (ArUco/QR) or shape.
    *   **Tech:** ArUco marker detection (very robust/fast).

20. **"Cleaner Supervisor"**
    *   **Goal:** Inspect floor after cleaning. "This spot is still dirty."
    *   **Tech:** Texture analysis / Anomaly detection.
