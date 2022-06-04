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
import cv2
import math
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
        self._root.geometry("2735x1693")
        self._root.config(bg="white")
        self._root.wait_visibility(self._root)
        self._root.wm_attributes("-alpha",0.5)

        #self.var = tkinter.StringVar()
        #self.var.set("start")
        #self.label = tkinter.Label(self._root,textvariable=self.var,width=300)
        #self.label.pack()

        # Make window non-resizable:
        #self._root.resizable(0, 0)

        # Create canvas:
        self.display_x = 2735
        self.display_y = 1823
        lagori_pos_x = 2735/2.0
        lagori_pos_y = 1823/2.0 - 130
        lagori_size = 200
        self._canvas = tkinter.Canvas(self._root, bg='yellow', width="2735",height="1693")
        self._canvas.create_rectangle(lagori_pos_x - lagori_size/2, lagori_pos_y - lagori_size/2,lagori_pos_x + lagori_size/2, lagori_pos_y + lagori_size/2, fill="blue")
        grid_width = 5
        self._canvas.create_rectangle(0, lagori_pos_y - grid_width, self.display_x, lagori_pos_y + grid_width, fill="red")
        self._canvas.create_rectangle(lagori_pos_x - grid_width, 0, lagori_pos_x+grid_width, self.display_y, fill="red")
        
        self.cap = cv2.VideoCapture(14)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

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

        self._canvas.pack()

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
        #display_string = "x : " + str(v_x) + ", y : " + str(v_y)
        #self.var.set(display_string)

    def _show_img(self):
        _, img = self.cap.read()
        height = img.shape[0]
        width = img.shape[1]
        #cv2.imshow('image', img2)
        #key = cv2.waitKey(1)
        #img = cv2.imread(img_path)
        #h,w = img.shape[:2]
        DIM=(640, 360)
        K=numpy.array([[640, 0.0, 320], [0.0, 640, 180], [0.0, 0.0, 1.0]])
        D=numpy.array([[-0.12428544821092972], [1.241814186449534], [-12.14701944648527], [16.720427864073375]])

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, numpy.eye(3), K, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        w_ratio = 1.1
        # 変換前4点の座標　p1:左上　p2:右上 p3:左下 p4:左下
        p1 = numpy.array([45, 30])
        p2 = numpy.array([595,30])
        p3 = numpy.array([-650, 550])
        p4 = numpy.array([1290, 550])

        #　幅取得
        o_width = numpy.linalg.norm(p2 - p1)
        o_width = math.floor(o_width * w_ratio)

        #　高さ取得
        o_height = numpy.linalg.norm(p3 - p1)
        o_height = math.floor(o_height)

        # 変換前の4点
        src = numpy.float32([p1, p2, p3, p4])

        # 変換後の4点
        dst = numpy.float32([[0, 0],[o_width, 0],[0, o_height],[o_width, o_height]])

        # 変換行列
        M = cv2.getPerspectiveTransform(src, dst)

        # 射影変換・透視変換する
        output = cv2.warpPerspective(undistorted_img, M, (o_width, o_height))

        output2 = cv2.resize(output,(int(self.display_x*0.857),int(self.display_y)))
        #undistorted_img2 =  cv2.resize(undistorted_img,(int(self.display_x),int(self.display_y)))

        cv2.imshow('revision', output2)
        cv2.moveWindow('revision', int((self.display_x -self.display_x*0.857)/2.0),0)
        
        #img2 = cv2.resize(img,(int(self.display_x),int(self.display_y)))
        #cv2.imshow('image', img2)
        key = cv2.waitKey(1)
    
    def _publish_twist(self, event):
        self._send_motion()
        self._show_img()

def main():
    rospy.init_node('mouse_teleop')

    MouseTeleop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
