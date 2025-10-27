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
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JoyFeedback


class RumbleFeedbackPublisher(Node):

    def __init__(self):
        super().__init__('rumble_feedback')

        param_descriptor = ParameterDescriptor(description='Rumble intensity')
        self.declare_parameter('intensity', 0.4, param_descriptor)

        self.add_on_set_parameters_callback(self.parameter_change_callback)

        self.intensity = self.get_parameter('intensity').value

        self.publisher_ = self.create_publisher(JoyFeedback, 'joy/set_feedback', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_timer_callback)

    def parameter_change_callback(self, params):

        result = SetParametersResult()

        # Iterate over each parameter in this node
        for param in params:
            # Check the parameter's name and type
            if param.name == 'intensity' and param.type_ == Parameter.Type.DOUBLE:
                # This parameter has changed. Display the new value to the terminal.
                self.get_logger().info('Parameter intensity changed to: %1.2f' % param.value)
                # The parameter change was successfully handled.
                result.successful = True

                # Set new intensity and check the range
                self.intensity = param.value

                if param.value < 0.0:
                    self.intensity = 0.00
                if param.value > 1.0:
                    self.intensity = 1.00
                if param.value != self.intensity:
                    self.get_logger().info(f'Parameter out of range, set to: {self.intensity}')

        return result

    def publish_timer_callback(self):
        msg = JoyFeedback()
        msg.type = JoyFeedback.TYPE_RUMBLE
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
        print('Received keyboard interrupt!')
    except (ExternalShutdownException):
        print('Received external shutdown request!')
    finally:
        # Destroy the node explicitly, if not already destroyed by the garbage collector
        if rumble_feedback is not None:
            rumble_feedback.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
