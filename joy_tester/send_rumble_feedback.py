# Copyright 2025 RbSCR
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node
from rclpy.parameter_event_handler import ParameterEventHandler

from sensor_msgs.msg import JoyFeedback


class RumbleFeedbackPublisher(Node):

    def __init__(self):
        super().__init__('rumble_feedback')

        param_descriptor = ParameterDescriptor(description='Rumble intensity')
        self.declare_parameter('intensity', 0.4, param_descriptor)
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name='intensity',
            node_name='rumble_feedback',
            callback=self.param_callback,
        )

        self.intensity = self.get_parameter('intensity').value

        self.publisher_ = self.create_publisher(JoyFeedback, 'joy/set_feedback', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_timer_callback)

    def param_callback(self, p: rclpy.parameter.Parameter) -> None:
        param_value = rclpy.parameter.parameter_value_to_python(p.value)
        self.get_logger().info(f'Received a parameter update: {p.name}: {param_value}')

        self.intensity = param_value

        if param_value < 0.0:
            self.intensity = 0.00
        if param_value > 1.0:
            self.intensity = 1.00
        if param_value != self.intensity:
            self.get_logger().info(f'Parameter out of range, set to: {self.intensity}')

    def publish_timer_callback(self):
        msg = JoyFeedback()
        msg.type = 1
        msg.id = 0
        msg.intensity = self.intensity

        self.publisher_.publish(msg)
        self.get_logger().info('Sending feedback, intensity: "%1.2f"' % msg.intensity)


def main(args=None):

    try:
        rclpy.init(args=args)

        rumble_feedback = RumbleFeedbackPublisher()

        rclpy.spin(rumble_feedback)

    except (KeyboardInterrupt):
        pass
    finally:
        # Destroy the node explicitly, if not already destroyed by the garbage collector
        if rumble_feedback is not None:
            rumble_feedback.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
