#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import pyaudio
import threading
import queue
import time
from faster_whisper import WhisperModel

class WhisperSttNode(Node):
    """
    Local Speech-to-Text node using faster-whisper.
    Captures audio from microphone and publishes transcribed text to ~/text.
    """
    def __init__(self):
        super().__init__('whisper_stt_node')

        # 1. Parameters
        self.declare_parameter('model_size', 'base') # tiny, base, small
        self.declare_parameter('device', 'cpu')      # cpu or cuda
        self.declare_parameter('compute_type', 'int8')
        self.declare_parameter('energy_threshold', 0.01) # Silence threshold
        self.declare_parameter('silence_duration', 1.0)  # Seconds of silence to trigger transcription
        
        model_size = self.get_parameter('model_size').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.silence_limit = self.get_parameter('silence_duration').value

        # 2. Init Whisper
        self.get_logger().info(f"Loading Whisper model '{model_size}' on {device}...")
        self.model = WhisperModel(model_size, device=device, compute_type=compute_type)
        
        # 3. Audio Setup
        self.rate = 16000 # Whisper expects 16kHz
        self.chunk = 1024
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # 4. State
        self.audio_buffer = []
        self.is_recording = False
        self.last_speech_time = time.time()
        self.text_pub = self.create_publisher(String, '~/text', 10)

        # 5. Start Processing Thread
        self.thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info("Whisper STT Node ready. Listening...")

    def audio_loop(self):
        while rclpy.ok():
            try:
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                samples = np.frombuffer(data, dtype=np.float32)
                energy = np.sqrt(np.mean(samples**2))

                if energy > self.energy_threshold:
                    if not self.is_recording:
                        self.get_logger().info("Speech detected...")
                        self.is_recording = True
                    self.audio_buffer.append(samples)
                    self.last_speech_time = time.time()
                elif self.is_recording:
                    # Silence detected while recording
                    self.audio_buffer.append(samples)
                    if (time.time() - self.last_speech_time) > self.silence_limit:
                        # End of segment reached
                        self.get_logger().info("End of speech. Transcribing...")
                        self.process_segment()
                        self.is_recording = False
                        self.audio_buffer = []
                
            except Exception as e:
                self.get_logger().error(f"Audio capture error: {e}")

    def process_segment(self):
        if not self.audio_buffer:
            return

        # Combine chunks into one array
        full_audio = np.concatenate(self.audio_buffer)
        
        # Whisper transcription
        segments, info = self.model.transcribe(full_audio, beam_size=5)
        
        full_text = ""
        for segment in segments:
            full_text += segment.text + " "
        
        full_text = full_text.strip()
        if full_text:
            self.get_logger().info(f"Result: '{full_text}' (lang: {info.language})")
            msg = String()
            msg.data = full_text
            self.text_pub.publish(msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperSttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
