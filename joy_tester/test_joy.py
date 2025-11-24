# Copyright 2023 Josh Newans
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

from tkinter import Canvas
from tkinter import Tk
from tkinter import ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback


class JoyButton:
    def __init__(self, parent_canvas, left_space, inner_space, width, vert_space, height, i):
        self.parent_canvas = parent_canvas
        row = i % 8
        col = i // 8
        lf_text = left_space + (inner_space + width) * col
        lf_circle = left_space + inner_space + (inner_space + width) * col
        t = vert_space + (height + vert_space) * row

        self.parent_canvas.create_text(lf_text, t+height/2, text=str(i))
        self.circle_obj = self.parent_canvas.create_oval(lf_circle, t,
                                                         lf_circle+height, t+height,
                                                         width=1, fill='white')

    def update_value(self, value):
        if (value > 0):
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FF0000')
        else:
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FFFFFF')


class JoyAxis:
    def __init__(self, parent_canvas, left_space, inner_space, width, vert_space, height, i):
        self.parent_canvas = parent_canvas
        self.height = height
        self.width = width
        self.lf = left_space + inner_space
        self.t = vert_space + (height + vert_space) * i

        self.parent_canvas.create_text(left_space, self.t+height/2, text=str(i))
        dummy_value = 0.0
        ww = width * (dummy_value + 1)/2
        self.fill_obj = self.parent_canvas.create_rectangle(self.lf, self.t,
                                                            self.lf+ww, self.t+height,
                                                            width=0, fill='green')
        self.outline_obj = self.parent_canvas.create_rectangle(self.lf, self.t,
                                                               self.lf+width, self.t+height,
                                                               width=1, outline='black')
        self.val_txt = self.parent_canvas.create_text(self.lf+width+inner_space, self.t+height/2,
                                                      anchor='w', text=str(f'{dummy_value:.5f}'))

    def update_value(self, value):
        ww = self.width * (value + 1)/2
        self.parent_canvas.coords(self.fill_obj, self.lf, self.t, self.lf+ww, self.t+self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=str(f'{value:.5f}'))


class Feedback:
    def __init__(self, parent_canvas, left_space, inner_space, width,
                 vert_space, height, feedback_type):
        self.parent_canvas = parent_canvas
        self.height = height
        self.width = width
        self.lf = left_space + inner_space + 40
        self.t = vert_space + (height + vert_space) * feedback_type

        self.colors = ['green', 'orange', 'red', 'blue', 'yellow']
        self.color_index = -1

        if feedback_type == JoyFeedback.TYPE_LED:
            type_txt = 'Led'
        elif feedback_type == JoyFeedback.TYPE_RUMBLE:
            type_txt = 'Rumble'
        elif feedback_type == JoyFeedback.TYPE_BUZZER:
            type_txt = 'Buzzer'

        self.parent_canvas.create_text(left_space, self.t+height/2, anchor='w', text=type_txt)

        ww = 0.0
        self.fill_obj = self.parent_canvas.create_rectangle(self.lf, self.t,
                                                            self.lf+ww, self.t+height,
                                                            width=0, fill='green')
        self.outline_obj = self.parent_canvas.create_rectangle(self.lf, self.t,
                                                               self.lf+width, self.t+height,
                                                               width=1, outline='black')
        self.val_txt = self.parent_canvas.create_text(self.lf+width+inner_space,
                                                      self.t+height/2, anchor='w',
                                                      text=' ')
        self.time_txt = self.parent_canvas.create_text(self.lf+width+(4*inner_space),
                                                       self.t+height/2,
                                                       anchor='w', text='No message yet ')

    def update_value(self, value, time_stamp):
        self.color_index = (self.color_index + 1) % len(self.colors)
        fill_color = self.colors[self.color_index]
        ww = self.width * value
        secs, nsecs = time_stamp.seconds_nanoseconds()

        self.parent_canvas.itemconfigure(self.fill_obj, fill=fill_color)
        self.parent_canvas.coords(self.fill_obj, self.lf, self.t, self.lf+ww, self.t+self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=str(f'{value:.2f}'))
        self.parent_canvas.itemconfigure(self.time_txt, text=str(f'at {secs}.{nsecs}'))


class JoyTester(Node):

    def __init__(self):
        super().__init__('test_joy')
        self.get_logger().info('Testing Joystick ...')

        self.subscription_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 5)
        self.subscription_feedback = self.create_subscription(JoyFeedback,
                                                              'joy/set_feedback',
                                                              self.feedback_callback, 5)

        self.buttons = []
        self.axes = []
        self.joy_objects_initialised = False
        self.feedback = []

        self.create_tk_root()
        self.create_joy_frames()

        # Note [RbSCR]
        # When the device is known at startup (using a parameter (future functionality))
        # the button and axes widgets could be initiated here to prevent an 'empty' frame.
        # For example for a PS3:
        # self.create_joy_button_widgets(17)
        # self.create_joy_axes_widgets(6)
        # self.joy_objects_initialised = True

        self.create_feedback_frames()
        self.tk.update()

    def create_tk_root(self):
        """Create the tkinter root."""
        self.tk = Tk()
        self.tk.geometry('650x500+0+0')
        self.tk.resizable(False, False)
        self.tk.title('Joystick Testing')
        self.tk.rowconfigure(0, weight=1)       # row for joy_frame
        self.tk.rowconfigure(1, weight=1)       # row for feedback_frame
        self.tk.columnconfigure(0, weight=1)

    def create_joy_frames(self):
        """Create the frames for joy topic data."""
        self.joy_frame = ttk.Labelframe(self.tk, borderwidth=1, relief='solid',
                                        padding=5, text='  Joy topic   ')
        self.joy_frame.grid(column=0, row=0)

        self.joy_frame.rowconfigure(0, weight=1)
        self.joy_frame.columnconfigure(0, weight=1)     # column 0 for button_frame
        self.joy_frame.columnconfigure(1, weight=1)     # column 1 for axes_frame

        # Create frame and canvas for the 'joy'buttons
        self.button_frame = ttk.Labelframe(self.joy_frame, relief='flat', text=' Buttons ')
        self.button_frame.grid(column=0, row=0, sticky='NW')

        self.button_frame.rowconfigure(0, weight=1)
        self.button_frame.columnconfigure(0, weight=1)

        self.button_canvas = Canvas(self.button_frame, width=275, background='lightgray')
        self.button_canvas.grid(column=0, row=0)

        # Create frame and canvas for the 'joy'axes
        self.axes_frame = ttk.Labelframe(self.joy_frame, text=' Axes ', relief='flat')
        self.axes_frame.grid(column=1, row=0, sticky='NW')

        self.axes_frame.rowconfigure(0, weight=1)
        self.axes_frame.columnconfigure(0, weight=1)

        self.axes_canvas = Canvas(self.axes_frame, width=250, background='lightgray')
        self.axes_canvas.grid(column=0, row=0)

    def create_joy_button_widgets(self, button_count):
        """Create the widgets for the buttons in the joy message."""
        left_space = 10
        inner_space = 15
        width = 70
        height = 25
        v_space = 5
        for i in range(0, button_count):
            self.buttons.append(JoyButton(self.button_canvas, left_space, inner_space,
                                          width, v_space, height, i))

    def create_joy_axes_widgets(self, axes_count):
        """Create the widgets for the axes in the joy message."""
        left_space = 10
        inner_space = 15
        width = 100
        height = 25
        vert_space = 5
        for i in range(0, axes_count):
            self.axes.append(JoyAxis(self.axes_canvas, left_space, inner_space,
                                     width, vert_space, height, i))

    def joy_callback(self, joy_msg):
        """Handle a receipt of a joy message."""
        # Handle first receive
        if not self.joy_objects_initialised:
            button_count = len(joy_msg.buttons)
            self.create_joy_button_widgets(button_count)

            axes_count = len(joy_msg.axes)
            self.create_joy_axes_widgets(axes_count)

            self.joy_objects_initialised = True

        # Update Values
        for i, val in enumerate(joy_msg.buttons):
            self.buttons[i].update_value(val)

        for i, val in enumerate(joy_msg.axes):
            self.axes[i].update_value(val)

        # Redraw
        self.tk.update()
        return

    def create_feedback_frames(self):
        """Create the frame for the feedbackframes."""
        self.feedback_frame = ttk.Labelframe(self.tk,
                                             borderwidth=1, relief='solid',
                                             padding=5, text='  Feedback topic   ')
        self.feedback_frame.grid(column=0, row=1)

        self.feedback_frame.rowconfigure(0, weight=1)
        self.feedback_frame.columnconfigure(0, weight=1)     # column 0 for subscribe frame
        self.feedback_frame.columnconfigure(1, weight=1)     # column 1 for publish frame

        # create subscribe feedback frame
        self.subscribe_frame = ttk.Labelframe(self.feedback_frame,
                                              borderwidth=1, relief='solid',
                                              padding=10, text=' Subscribe feedback ')
        self.subscribe_frame.grid(column=0, row=0, sticky='NW')
        self.subscribe_canvas = Canvas(self.subscribe_frame, width=400, height=100,
                                       background='lightgray')
        self.subscribe_canvas.pack()

        left_space = 5
        inner_space = 15
        width = 100
        height = 25
        vert_space = 5
        self.feedback.append(Feedback(self.subscribe_canvas, left_space, inner_space,
                                      width, vert_space, height, JoyFeedback.TYPE_LED))
        self.feedback.append(Feedback(self.subscribe_canvas, left_space, inner_space,
                                      width, vert_space, height, JoyFeedback.TYPE_RUMBLE))
        self.feedback.append(Feedback(self.subscribe_canvas, left_space, inner_space,
                                      width, vert_space, height, JoyFeedback.TYPE_BUZZER))
        # Note [RbSCR]
        # Feedback widgets are immediately created (in contrast to the button and axis widgets)
        # because the number of widgets/feedback-types is known to be a fixed value.

    def feedback_callback(self, set_feedback_msg):
        """Handle a receipt of a set_feedback message."""
        t_stamp = self.get_clock().now()
        self.feedback[set_feedback_msg.type].update_value(set_feedback_msg.intensity, t_stamp)

        # Redraw
        self.tk.update()
        return

        # Note [RbSCR]
        # Code for the publish frame and widgets for publishing feedback
        # messages should go here. But 'ROS spin' and 'Tk mainloop' don't
        # work together (mainloop is needed for the UI interaction).
        # A possible solution is to use asyncio.


def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()

    try:
        rclpy.spin(joy_tester)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')

    print('Exiting...')

    # and destry the ros node
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
