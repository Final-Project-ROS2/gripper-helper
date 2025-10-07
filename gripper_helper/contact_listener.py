import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
import time

class ContactListener(Node):
    def __init__(self):
        super().__init__('contact_listener')

        # Subscribe to /contact
        self.contact_sub = self.create_subscription(
            String,
            '/contact',
            self.contact_callback,
            10)

        # Subscribe to /gripper/monitor
        self.monitor_sub = self.create_subscription(
            String,
            '/gripper/monitor',
            self.monitor_callback,
            10)

        # Service clients
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        # Internal state
        self.gripped = False
        self.model1_name = None
        self.link1_name = None
        self.model2_name = None
        self.link2_name = None

        # Timestamp to temporarily ignore contact messages
        self.ignore_contact_until = 0.0  # Unix timestamp until which contact messages are ignored

        self.get_logger().info('Contact listener started. Waiting for /contact and /gripper/monitor messages...')

    def contact_callback(self, msg):
        # Ignore contact messages if within ignore window
        if time.time() < self.ignore_contact_until:
            self.get_logger().info('Ignoring /contact message temporarily after detach.')
            return

        if self.gripped:
            return

        model1_name, link1_name, model2_name, link2_name = self.parse_contact_msg(msg.data)
        if model1_name is None:
            return

        if link1_name == 'robotiq_85_left_finger_tip_link':
            self.get_logger().info("Replacing link1 with wrist_3_link.")
            link1_name = 'wrist_3_link'
        elif link2_name == 'robotiq_85_left_finger_tip_link':
            self.get_logger().info("Replacing link2 with wrist_3_link.")
            link2_name = 'wrist_3_link'

        self.get_logger().info(f"Parsed contact — model1: {model1_name}, link1: {link1_name}, model2: {model2_name}, link2: {link2_name}")

        # Store values for detach
        self.model1_name = model1_name
        self.link1_name = link1_name
        self.model2_name = model2_name
        self.link2_name = link2_name

        self.call_attach_service(model1_name, link1_name, model2_name, link2_name)

    def monitor_callback(self, msg):
        if not self.gripped:
            return

        # Parse position from the monitor string
        try:
            parts = [p.strip() for p in msg.data.replace(',', '').split()]
            pos_index = parts.index('position:') + 1
            position = float(parts[pos_index])
        except Exception as e:
            self.get_logger().error(f"Failed to parse /gripper/monitor message: {msg.data} ({e})")
            return

        # Only detach if position is 0.0
        if position != 0.0:
            self.get_logger().info(f"Monitor position is {position}, not zero. Doing nothing.")
            return

        self.get_logger().info('Monitor position is 0.0, calling /DETACHLINK...')

        # Temporarily ignore /contact messages for 3 seconds
        self.ignore_contact_until = time.time() + 3.0

        self.call_detach_service(
            self.model1_name,
            self.link1_name,
            self.model2_name,
            self.link2_name
        )

    def parse_contact_msg(self, data):
        try:
            parts = [p.strip() for p in data.replace(',', '').split()]
            model1_name = parts[parts.index('model1_name:') + 1]
            link1_name = parts[parts.index('link1_name:') + 1]
            model2_name = parts[parts.index('model2_name:') + 1]
            link2_name = parts[parts.index('link2_name:') + 1]
            return model1_name, link1_name, model2_name, link2_name
        except Exception as e:
            self.get_logger().error(f"Failed to parse contact message: {data} ({e})")
            return None, None, None, None

    def call_attach_service(self, model1, link1, model2, link2):
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/ATTACHLINK service not available')
            return

        req = AttachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2

        future = self.attach_client.call_async(req)
        future.add_done_callback(self.attach_response_callback)

    def attach_response_callback(self, future):
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.gripped = True
                self.get_logger().info('Successfully attached links! Gripped set to True.')
            else:
                self.get_logger().warn('AttachLink service call failed or returned False.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def call_detach_service(self, model1, link1, model2, link2):
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/DETACHLINK service not available')
            return

        req = DetachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2

        future = self.detach_client.call_async(req)
        future.add_done_callback(self.detach_response_callback)

    def detach_response_callback(self, future):
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.gripped = False
                self.get_logger().info('Successfully detached links! Gripped set to False.')
            else:
                self.get_logger().warn('DetachLink service call failed or returned False.')
        except Exception as e:
            self.get_logger().error(f'DetachLink service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ContactListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
