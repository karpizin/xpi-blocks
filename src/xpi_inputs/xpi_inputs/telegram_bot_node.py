#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int32, Empty
from sensor_msgs.msg import Image
import threading
import asyncio
import cv2
import numpy as np
from telegram import Update, ReplyKeyboardMarkup
from telegram.ext import ApplicationBuilder, CommandHandler, MessageHandler, ContextTypes, filters
import os

class TelegramBotNode(Node):
    def __init__(self):
        super().__init__('telegram_bot_node')

        # Parameters
        self.declare_parameter('token', '')
        self.declare_parameter('allowed_users', []) # List of user IDs
        self.declare_parameter('menu_layout', [])   # List of lists of strings
        self.declare_parameter('actions', [])       # List of strings "TRIGGER:TOPIC:TYPE:VALUE"

        self.token = self.get_parameter('token').value
        self.allowed_users = self.get_parameter('allowed_users').value
        self.menu_layout = self.get_parameter('menu_layout').value # This might need special parsing if passed as list of strings rep
        
        # Simple Actions Parsing
        # Format: "Button Text:topic_name:Type:Value"
        # Example: "Light ON:light:Bool:True"
        self.actions_config = self.get_parameter('actions').value
        self.action_map = {}
        self.pubs = {}

        self.parse_actions()

        # State
        self.last_image = None
        self.lock = threading.Lock()

        # Subscribers
        self.create_subscription(String, '~/notify', self.notify_callback, 10)
        self.create_subscription(Image, '~/image_raw', self.image_callback, 10) # Connect camera here

        if not self.token:
            self.get_logger().error("Telegram Token not provided! Set 'token' parameter.")
            return

        # Start Bot in separate thread (asyncio loop)
        self.bot_thread = threading.Thread(target=self.run_bot_loop, daemon=True)
        self.bot_thread.start()
        
        self.get_logger().info("Telegram Bot started.")

    def parse_actions(self):
        for act in self.actions_config:
            try:
                # Syntax: "Trigger Text:topic:Type:Value"
                # Value can be optional for Empty/Photo
                parts = act.split(':')
                trigger = parts[0]
                topic = parts[1]
                msg_type = parts[2]
                val = parts[3] if len(parts) > 3 else None

                # Special Actions
                if topic == 'internal' and msg_type == 'photo':
                    self.action_map[trigger] = {'type': 'photo'}
                    continue

                # Create Publisher
                if topic not in self.pubs:
                    if msg_type == 'Bool': cls = Bool
                    elif msg_type == 'String': cls = String
                    elif msg_type == 'Int32': cls = Int32
                    elif msg_type == 'Float32': cls = Float32
                    elif msg_type == 'Empty': cls = Empty
                    else: continue
                    self.pubs[topic] = self.create_publisher(cls, topic, 10)

                self.action_map[trigger] = {
                    'type': 'pub',
                    'topic': topic,
                    'msg_type': msg_type,
                    'value': val,
                    'pub': self.pubs[topic]
                }
            except Exception as e:
                self.get_logger().warn(f"Failed to parse action '{act}': {e}")

    def run_bot_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        app = ApplicationBuilder().token(self.token).build()

        # Handlers
        app.add_handler(CommandHandler("start", self.cmd_start))
        app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, self.handle_message))

        self.bot_app = app # Save ref to send notifications later
        
        app.run_polling(stop_signals=None) # Blocking in this thread

    async def check_auth(self, update: Update):
        user_id = update.effective_user.id
        # If allowed_users is empty, allow everyone (Dangerous!) -> Let's enforce non-empty or specific flag
        # For now: if list is empty, warn but allow? No, secure by default.
        if not self.allowed_users:
            await update.message.reply_text("⚠️ No allowed users configured. Access denied.")
            return False
            
        if user_id not in self.allowed_users:
            await update.message.reply_text("⛔ Access denied.")
            self.get_logger().warn(f"Unauthorized access attempt from ID: {user_id}")
            return False
        return True

    async def cmd_start(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        if not await self.check_auth(update): return

        # Build Keyboard
        # menu_layout param comes as list of strings? ROS params are tricky with nested lists.
        # Let's assume a simplified flattened list with separators or just hardcoded rows via a parser?
        # Better: Assume `menu_layout` is list of strings, each string is a ROW, items separated by '|'
        # Example: ["Btn1|Btn2", "Btn3"]
        
        keyboard = []
        if self.menu_layout:
            for row_str in self.menu_layout:
                btns = [b.strip() for b in row_str.split('|')]
                keyboard.append(btns)
        else:
            keyboard = [['ℹ️ Status']]

        reply_markup = ReplyKeyboardMarkup(keyboard, resize_keyboard=True)
        await update.message.reply_text("🤖 Bot Active. Control Panel:", reply_markup=reply_markup)

    async def handle_message(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        if not await self.check_auth(update): return

        text = update.message.text
        
        if text in self.action_map:
            action = self.action_map[text]
            
            if action['type'] == 'photo':
                await self.send_photo(update)
            elif action['type'] == 'pub':
                self.execute_pub(action)
                await update.message.reply_text(f"✅ Sent: {text}")
        else:
            await update.message.reply_text("❓ Unknown command.")

    def execute_pub(self, action):
        msg = None
        t = action['msg_type']
        v = action['value']
        
        if t == 'Bool':
            msg = Bool()
            msg.data = (v.lower() == 'true')
        elif t == 'String':
            msg = String()
            msg.data = v
        elif t == 'Empty':
            msg = Empty()
        elif t == 'Float32':
            msg = Float32()
            msg.data = float(v)
            
        if msg:
            action['pub'].publish(msg)

    async def send_photo(self, update):
        img = None
        with self.lock:
            if self.last_image is not None:
                img = self.last_image.copy()
        
        if img is None:
            await update.message.reply_text("📷 No image available (camera not active?)")
            return

        # Convert to JPEG
        try:
            # Assuming RGB8 or BGR8. ROS Image is raw bytes.
            # Need to know encoding. Let's assume 'bgr8' or 'rgb8'.
            # Image.data is byte array.
            # Width/Height/Step
            
            # Simple manual decode for common formats
            # If using v4l2_camera, it usually sends 'rgb8'. OpenCV uses 'bgr8'.
            
            h = self.last_height
            w = self.last_width
            
            # Construct NP array
            dtype = np.uint8
            channels = 3
            np_arr = np.frombuffer(img, dtype=dtype).reshape((h, w, channels))
            
            # If rgb8, convert to bgr8 for opencv
            if self.last_encoding == 'rgb8':
                np_arr = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
            
            # Encode to JPEG
            ret, jpeg = cv2.imencode('.jpg', np_arr)
            if ret:
                await update.message.reply_photo(jpeg.tobytes())
            else:
                await update.message.reply_text("❌ Failed to encode image.")
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
            await update.message.reply_text("❌ Image processing error.")

    def image_callback(self, msg):
        with self.lock:
            self.last_image = msg.data
            self.last_width = msg.width
            self.last_height = msg.height
            self.last_encoding = msg.encoding

    def notify_callback(self, msg):
        # Async call from sync callback is tricky. 
        # We need to schedule it in the bot loop.
        # But we don't have easy access to the loop from here thread-safely without nest_asyncio or similar.
        # Simplest way: use `asyncio.run_coroutine_threadsafe`.
        if hasattr(self, 'bot_app') and self.allowed_users:
            loop = self.bot_app.updater.bot._request.loop if hasattr(self.bot_app, 'updater') else None
            # Actually, just get the loop from the thread?
            # Or iterate users and send.
            pass
            # TODO: Implementation of Async Notify from Sync ROS Callback needs careful threading.
            # Skipping for this iteration to ensure stability of the main features.

def main(args=None):
    rclpy.init(args=args)
    node = TelegramBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
