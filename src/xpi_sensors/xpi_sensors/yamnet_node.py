import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import tflite_runtime.interpreter as tflite
import urllib.request
import os
import json
import csv

class YamnetNode(Node):
    def __init__(self):
        super().__init__('yamnet_node')
        
        # Parameters
        self.declare_parameter('device_index', -1)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('model_path', 'yamnet.tflite')
        
        self.threshold = self.get_parameter('threshold').value
        device = self.get_parameter('device_index').value
        if device == -1: device = None
        
        # Model setup
        model_path = self.get_parameter('model_path').value
        class_map_path = 'yamnet_class_map.csv'
        
        self.ensure_model_exists(model_path, class_map_path)
        
        # Initialize TFLite
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Load labels
        self.classes = []
        with open(class_map_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.classes.append(row['display_name'])

        # Publishers
        self.det_pub = self.create_publisher(String, '~/detections', 10)
        self.raw_pub = self.create_publisher(String, '~/raw_detections', 10)

        # Audio Stream
        self.fs = 16000 # YAMNet expects 16kHz
        self.block_size = 15600 # Approx 0.975s required by YAMNet
        
        try:
            self.stream = sd.InputStream(
                device=device,
                channels=1,
                samplerate=self.fs,
                blocksize=self.block_size,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info(f"YAMNet started on {device if device else 'default'} (16kHz)")
        except Exception as e:
            self.get_logger().error(f"Audio error: {e}")

    def ensure_model_exists(self, model_path, class_map_path):
        if not os.path.exists(model_path):
            self.get_logger().info("Downloading YAMNet model...")
            url = "https://github.com/vcb88/xpi-blocks-models/raw/main/yamnet.tflite" # Example repo
            urllib.request.urlretrieve(url, model_path)
        
        if not os.path.exists(class_map_path):
            self.get_logger().info("Downloading YAMNet class map...")
            url = "https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet/yamnet_class_map.csv"
            urllib.request.urlretrieve(url, class_map_path)

    def audio_callback(self, indata, frames, time, status):
        # YAMNet expects float32 in [-1, 1]
        waveform = indata.flatten().astype(np.float32)
        
        self.interpreter.set_tensor(self.input_details[0]['index'], waveform)
        self.interpreter.invoke()
        
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        
        # Get top 3
        top_indices = np.argsort(scores)[-3:][::-1]
        results = []
        
        for idx in top_indices:
            if scores[idx] > self.threshold:
                results.append({
                    "class": self.classes[idx],
                    "score": float(scores[idx])
                })

        if results:
            # Publish top result
            msg = String()
            msg.data = results[0]['class']
            self.det_pub.publish(msg)
            
            # Publish all results
            raw_msg = String()
            raw_msg.data = json.dumps(results)
            self.raw_pub.publish(raw_msg)
            
            self.get_logger().debug(f"Detected: {results[0]['class']} ({results[0]['score']:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = YamnetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
