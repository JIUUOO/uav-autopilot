import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimDetectionEventNode(Node):
    def __init__(self):
        super().__init__("sim_detection_event_node")

        self.declare_parameter("publish_hz", 0.1)

        self.publisher = self.create_publisher(String, "/sim/detection/event", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.publish_event)

        self.count = 0

        self.get_logger().info("sim_detection_event_node started")

    def publish_event(self):
        msg = String()
        msg.data = (
            "{"
            f'"event_id":"sim_event_{self.count:04d}",'
            '"event_type":"object_detected",'
            '"object_class":"person",'
            '"confidence":0.87,'
            '"action_recommendation":"hold_and_report"'
            "}"
        )

        self.publisher.publish(msg)
        self.get_logger().info(f"published fake detection event {self.count}")

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimDetectionEventNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()