import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import time


class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor_node')

        # Last time we received a /contact message
        self.last_contact_time = 0.0
        self.timeout_sec = 1.0  # If no contact within 1s, considered False

        # Subscribe to /contact topic
        self.create_subscription(String, '/contact', self.contact_callback, 10)

        # Service server for /is_contact
        self.srv = self.create_service(SetBool, '/is_contact', self.is_contact_callback)

        self.get_logger().info('Contact monitor node started.')

    def contact_callback(self, msg):
        self.last_contact_time = time.time()
        self.get_logger().debug(f"Received contact message: {msg.data}")

    def is_contact_callback(self, request, response):
        """Return True if contact message received recently."""
        now = time.time()
        contact_active = (now - self.last_contact_time) < self.timeout_sec

        response.success = contact_active
        response.message = "Contact active" if contact_active else "No recent contact"
        self.get_logger().info(f"/is_contact called → returning {contact_active}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
