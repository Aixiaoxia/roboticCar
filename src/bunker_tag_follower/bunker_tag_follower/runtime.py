"""Keep the ROS context alive long enough to publish shutdown stop messages."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions


def initialize_runtime():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
