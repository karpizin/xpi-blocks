import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import usb.core
import usb.util
import time

# ReSpeaker USB Vendor and Product ID
VID = 0x2886
PID = 0x0018

class RespeakerNode(Node):
    def __init__(self):
        super().__init__('respeaker_node')
        
        self.declare_parameter('update_rate', 0.1) # seconds
        update_rate = self.get_parameter('update_rate').value

        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if not self.dev:
            self.get_logger().error("ReSpeaker Mic Array v2.0 not found. Check USB connection.")
            # We don't raise here to allow the node to stay alive, but it won't work
            self.active = False
        else:
            self.get_logger().info("ReSpeaker Mic Array v2.0 connected.")
            self.active = True

        # Publishers
        self.doa_pub = self.create_publisher(Int32, '~/doa', 10)
        self.vad_pub = self.create_publisher(Int32, '~/vad', 10)
        self.speech_pub = self.create_publisher(Bool, '~/is_speech', 10)

        # Timer for polling parameters from the DSP
        if self.active:
            self.timer = self.create_timer(update_rate, self.timer_callback)

    def write(self, id, data):
        """Write parameter to the DSP."""
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, id, 0, data)

    def read(self, id, type='int'):
        """Read parameter from the DSP."""
        res = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, id, 0, 8)
        
        if type == 'int':
            return int(res[0]) | (int(res[1]) << 8) | (int(res[2]) << 16) | (int(res[3]) << 24)
        return res

    def timer_callback(self):
        try:
            # DOA (Direction of Arrival) parameter ID is 21
            doa_angle = self.read(21)
            
            # VAD (Voice Activity Detection) parameter ID is 19
            vad_state = self.read(19)

            # Publish DOA
            msg_doa = Int32()
            msg_doa.data = doa_angle
            self.doa_pub.publish(msg_doa)

            # Publish VAD
            msg_vad = Int32()
            msg_vad.data = vad_state
            self.vad_pub.publish(msg_vad)

            # Publish Boolean is_speech
            msg_speech = Bool()
            msg_speech.data = True if vad_state == 1 else False
            self.speech_pub.publish(msg_speech)

            self.get_logger().debug(f"DOA: {doa_angle}, VAD: {vad_state}")

        except Exception as e:
            self.get_logger().error(f"Error reading from ReSpeaker: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RespeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
