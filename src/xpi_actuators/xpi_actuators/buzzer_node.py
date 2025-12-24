import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from gpiozero import TonalBuzzer, Device
from gpiozero.tones import Tone
from gpiozero.pins.mock import MockFactory
import time
import threading

class BuzzerNode(Node):
    """
    ROS2 Node for an active or passive buzzer.
    Supports simple beeps and RTTTL melodies.
    """

    # Note frequencies for RTTTL (4th octave)
    NOTES = {
        'c': 261.63, 'c#': 277.18, 'd': 293.66, 'd#': 311.13, 'e': 329.63, 'f': 349.23,
        'f#': 369.99, 'g': 392.00, 'g#': 415.30, 'a': 440.00, 'a#': 466.16, 'b': 493.88, 'p': 0
    }

    def __init__(self):
        super().__init__('buzzer_node')

        # Parameters
        self.declare_parameter('gpio_pin', 18)
        self.declare_parameter('is_passive', True)
        self.declare_parameter('mock_hardware', False)

        pin_num = self.get_parameter('gpio_pin').value
        self.is_passive = self.get_parameter('is_passive').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('Buzzer: Running in MOCK mode.')

        # Hardware Setup
        try:
            self.buzzer = TonalBuzzer(pin_num)
            self.get_logger().info(f"Buzzer initialized on GPIO {pin_num}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Buzzer: {e}")
            raise e

        # Publishers & Subscribers
        self.create_subscription(Bool, '~/beep', self.beep_callback, 10)
        self.create_subscription(String, '~/melody', self.melody_callback, 10)

        self.play_thread = None
        self.stop_requested = False

    def beep_callback(self, msg):
        if msg.data:
            self.buzzer.play(Tone(440)) # A4
        else:
            self.buzzer.stop()

    def melody_callback(self, msg):
        """Starts a background thread to play the RTTTL melody."""
        if self.play_thread and self.play_thread.is_alive():
            self.get_logger().warn("Melody already playing. Ignoring.")
            return
        
        self.stop_requested = False
        self.play_thread = threading.Thread(target=self.play_rtttl, args=(msg.data,))
        self.play_thread.start()

    def play_rtttl(self, rtttl_str):
        """Minimal RTTTL parser and player."""
        try:
            # Format: name:settings:notes
            parts = rtttl_str.split(':')
            if len(parts) < 3: return
            
            name = parts[0]
            settings = parts[1].split(',')
            notes = parts[2].split(',')

            # Defaults
            duration = 4
            octave = 5
            bpm = 63

            for s in settings:
                if s.startswith('d='): duration = int(s[2:])
                if s.startswith('o='): octave = int(s[2:])
                if s.startswith('b='): bpm = int(s[2:])

            beat_duration = 60.0 / bpm
            self.get_logger().info(f"Playing melody: {name}")

            for n in notes:
                if self.stop_requested: break
                
                # Parse note string (e.g., "8c#6")
                n = n.strip().lower()
                
                # 1. Duration
                note_dur = ""
                while n and n[0].isdigit():
                    note_dur += n[0]
                    n = n[1:]
                d = int(note_dur) if note_dur else duration
                
                # 2. Key
                key = n[0]
                n = n[1:]
                if n and n[0] == '#':
                    key += '#'
                    n = n[1:]
                
                # 3. Dot (prolongation)
                dot = False
                if n and n[0] == '.':
                    dot = True
                    n = n[1:]
                
                # 4. Octave
                o = int(n[0]) if n and n[0].isdigit() else octave

                # Calculate frequency
                freq = self.NOTES.get(key, 0)
                if freq > 0:
                    # Shift frequency based on octave
                    freq = freq * (2 ** (o - 4))
                
                # Calculate time
                play_time = (beat_duration * 4.0) / d
                if dot: play_time *= 1.5

                if freq > 0:
                    self.buzzer.play(Tone(freq))
                else:
                    self.buzzer.stop()
                
                time.sleep(play_time * 0.9) # Note duration
                self.buzzer.stop()
                time.sleep(play_time * 0.1) # Pause between notes

        except Exception as e:
            self.get_logger().error(f"Error playing RTTTL: {e}")
        finally:
            self.buzzer.stop()

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_requested = True
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
