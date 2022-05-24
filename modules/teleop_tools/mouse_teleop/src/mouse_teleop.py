#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2015 Enrique Fernandez
# Released under the BSD License.
#
# Authors:
#   * Enrique Fernandez

try:
    import tkinter
except ImportError:
    import Tkinter as tkinter

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

import pyautogui

import numpy
#import cv2
#from PIL import Image, ImageTk, ImageOps


class MouseTeleop():
    def __init__(self):
        # Retrieve params:
        self._frequency = rospy.get_param('~frequency', 50.0)
        self._scale = rospy.get_param('~scale', 1.0)
        self._holonomic = rospy.get_param('~holonomic', False)

        # Create twist publisher:
        self._pub_cmd = rospy.Publisher('/mouse_vel', Twist, queue_size=10)
        self._pub_key = rospy.Publisher('/keypress', String, queue_size=10)
        self._pub_key_release = rospy.Publisher('/keyrelease', String, queue_size=10)

        # Initialize twist components to zero:
        self._x = 0.0
        self._y = 0.0
        self._recent_x = 2735.0 / 2.0
        self._recent_y = 1823.0 / 2.0
        self._enable_ctrl = False

        # Initialize mouse position (x, y) to None (unknown); it's initialized
        # when the mouse button is pressed on the _start callback that handles
        # that event:

        # Create window:
        self._root = tkinter.Tk()
        self._root.title('Mouse Teleop')

        # Make window non-resizable:
        self._root.resizable(0, 0)

        # Create canvas:
        #self._canvas = tkinter.Canvas(self._root, bg='white', width="1300",height="100")

        # Create canvas objects:
        #self._canvas.create_arc(0, 0, 0, 0, fill='red', outline='red',
        #        width=1, style=tkinter.PIESLICE, start=90.0, tag='w')
        #self._canvas.create_line(0, 0, 0, 0, fill='blue', width=4, tag='v_x')

        #if self._holonomic:
        #    self._canvas.create_line(0, 0, 0, 0,
        #            fill='blue', width=4, tag='v_y')

        # Create canvas text objects:
        #self._text_v_x = tkinter.StringVar()
        #if self._holonomic:
        #    self._text_v_y = tkinter.StringVar()
        #self._text_w   = tkinter.StringVar()
#
        #self._label_v_x = tkinter.Label(self._root,
        #        anchor=tkinter.W, textvariable=self._text_v_x)
        #if self._holonomic:
        #    self._label_v_y = tkinter.Label(self._root,
        #            anchor=tkinter.W, textvariable=self._text_v_y)
        #self._label_w = tkinter.Label(self._root,
        #        anchor=tkinter.W, textvariable=self._text_w)
#
        #if self._holonomic:
        #    self._text_v_x.set('v_x = %0.2f m/s' % self._v_x)
        #    self._text_v_y.set('v_y = %0.2f m/s' % self._v_y)
        #    self._text_w.set(  'w   = %0.2f deg/s' % self._w)
        #else:
        #    self._text_v_x.set('v = %0.2f m/s' % self._v_x)
        #    self._text_w.set(  'w = %0.2f deg/s' % self._w)
#
        #self._label_v_x.pack()
        #if self._holonomic:
        #    self._label_v_y.pack()
        #self._label_w.pack()

        # Bind event handlers:
        self._root.bind('<KeyPress>', self._keypress)
        self._root.bind('<KeyRelease>', self._keyrelease)
        #self._root.bind('<KeyPress-s>', self._release)

        #self._canvas.bind('<Configure>', self._configure)

        #if self._holonomic:
        #    self._canvas.bind('<B1-Motion>', self._mouse_motion_linear)
        #    self._canvas.bind('<Shift-B1-Motion>', self._mouse_motion_angular)
#
        #    self._root.bind('<Shift_L>', self._change_to_motion_angular)
        #    self._root.bind('<KeyRelease-Shift_L>',
        #            self._change_to_motion_linear)
        #else:
        #    self._canvas.bind('<B1-Motion>', self._mouse_motion_angular)

        #self._canvas.pack()

        # If frequency is positive, use synchronous publishing mode:
        if self._frequency > 0.0:
            # Create timer for the given frequency to publish the twist:
            period = rospy.Duration(1.0 / self._frequency)

            self._timer = rospy.Timer(period, self._publish_twist)

        # Start window event manager main loop:
        self._root.mainloop()

    def __del__(self):
        if self._frequency > 0.0:
            self._timer.shutdown()

        self._root.quit()

    def _keypress(self, event):
        pressedKey = event.keysym
        #rospy.loginfo(pressedKey)
        if pressedKey == "x":
            if self._enable_ctrl is False:
                self._enable_ctrl = True
                #rospy.loginfo("pushed a")
                try:
                    pyautogui.moveTo(self._recent_x,self._recent_y)
                except:
                    rospy.loginfo("mouse move error")
        elif pressedKey == "s":
            self._enable_ctrl = False
            self._recent_x, self._recent_y = self._x, self._y
        
        pub_key = String(pressedKey)
        self._pub_key.publish(pub_key)

    def _keyrelease(self, event):
        releasedKey = event.keysym
        pub_key_release = String(releasedKey)
        self._pub_key_release.publish(pub_key_release)
        

    def _start(self, event):
        #self._x, self._y = pyautogui.position()
        if self._enable_ctrl is False:
            self._enable_ctrl = True
            rospy.loginfo("pushed a")
            try:
                pyautogui.moveTo(self._recent_x,self._recent_y)
            except:
                rospy.loginfo("mouse move error")

    def _release(self, event):
        self._enable_ctrl = False
        self._recent_x, self._recent_y = self._x, self._y

    def _send_motion(self):
        try:
            self._x ,self._y = pyautogui.position()
        except:
            rospy.loginfo("mouse data get error")
            self._x ,self._y = 0.0, 0.0

        if self._enable_ctrl == True:
            v_x = self._x
            v_y = self._y  
        else:
            v_x = self._recent_x
            v_y = self._recent_y

        linear  = Vector3(v_x, v_y, 0.0)
        angular = Vector3(0.0, 0.0, 0.0)

        twist = Twist(linear, angular)
        self._pub_cmd.publish(twist)

    def _publish_twist(self, event):
        self._send_motion()

def main():
    rospy.init_node('mouse_teleop')

    MouseTeleop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
