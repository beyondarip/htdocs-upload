#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import cv2
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, 
                                       QHBoxLayout, QGridLayout, QFrame, QGroupBox)
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap, QFont, QPalette, QColor
from sensor_msgs.msg import Image, NavSatFix
from std_srvs.srv import SetBool
import numpy as np

class ModernStyle:
    """Modern UI styling dengan light theme"""
    # Color palette
    COLORS = {
        'background': '#ffffff',
        'surface': '#f8fafc',
        'primary': '#3b82f6',
        'secondary': '#6366f1',
        'success': '#10b981',
        'error': '#ef4444',
        'warning': '#f59e0b',
        'text': '#1e293b',
        'text_dim': '#64748b',
        'border': '#e2e8f0'
    }

    # Styles
    BUTTON_STYLE = """
        QPushButton {
            background-color: #3b82f6;
            color: white;
            border: none;
            border-radius: 8px;
            padding: 10px 20px;
            font-weight: bold;
            font-size: 13px;
            min-width: 150px;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1d4ed8;
        }
        QPushButton[recording="true"] {
            background-color: #ef4444;
        }
        QPushButton[recording="true"]:hover {
            background-color: #dc2626;
        }
    """

    FRAME_STYLE = """
        QFrame {
            background-color: #f8fafc;
            border: 1px solid #e2e8f0;
            border-radius: 12px;
            padding: 12px;
        }
    """

    GROUP_BOX_STYLE = """
        QGroupBox {
            background-color: #ffffff;
            border: 1px solid #e2e8f0;
            border-radius: 8px;
            margin-top: 12px;
            font-weight: bold;
            padding: 15px;
        }
        QGroupBox::title {
            color: #1e293b;
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
        }
    """

    LABEL_STYLE = """
        QLabel {
            color: #1e293b;
            font-size: 13px;
        }
    """

    GPS_LABEL_STYLE = """
        QLabel {
            color: #1e293b;
            font-size: 14px;
            font-weight: bold;
            padding: 10px;
            background-color: #f8fafc;
            border: 1px solid #e2e8f0;
            border-radius: 8px;
        }
    """

class GPSCameraPlugin(Plugin):
    def __init__(self, context):
        super(GPSCameraPlugin, self).__init__(context)
        self.setObjectName('GPSCameraPlugin')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName('GPSCameraPluginUi')
        self._widget.setStyleSheet("background-color: white;")
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Camera Section
        camera_group = QGroupBox("Camera Feeds")
        camera_group.setStyleSheet(ModernStyle.GROUP_BOX_STYLE)
        camera_layout = QGridLayout()
        camera_layout.setSpacing(15)
        
        self.camera_frames = {}
        for i in range(3):
            frame = QFrame()
            frame.setStyleSheet(ModernStyle.FRAME_STYLE)
            frame_layout = QVBoxLayout()
            
            title = QLabel("Camera {}".format(i))
            title.setStyleSheet(ModernStyle.LABEL_STYLE)
            title.setAlignment(Qt.AlignCenter)
            frame_layout.addWidget(title)
            
            self.camera_frames[i] = QLabel()
            self.camera_frames[i].setMinimumSize(320, 240)
            self.camera_frames[i].setAlignment(Qt.AlignCenter)
            self.camera_frames[i].setStyleSheet("border: 1px solid #e2e8f0; border-radius: 8px;")
            frame_layout.addWidget(self.camera_frames[i])
            
            frame.setLayout(frame_layout)
            camera_layout.addWidget(frame, 0, i)
            
        camera_group.setLayout(camera_layout)
        main_layout.addWidget(camera_group)
        
        # Status Section
        status_group = QGroupBox("System Status")
        status_group.setStyleSheet(ModernStyle.GROUP_BOX_STYLE)
        status_layout = QHBoxLayout()
        
        self.gps_label = QLabel("GPS: Waiting for data...")
        self.gps_label.setStyleSheet(ModernStyle.GPS_LABEL_STYLE)
        status_layout.addWidget(self.gps_label)
        
        self.record_button = QPushButton("Start Recording")
        self.record_button.setStyleSheet(ModernStyle.BUTTON_STYLE)
        self.record_button.clicked.connect(self.toggle_recording)
        self.record_button.setProperty("recording", False)
        status_layout.addWidget(self.record_button)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        self._widget.setLayout(main_layout)
        context.add_widget(self._widget)
        
        # ROS subscribers
        self.camera_subs = []
        for i in range(3):
            sub = rospy.Subscriber('/camera{}/image_raw'.format(i), Image,
                                 lambda msg, cam_id=i: self.camera_callback(msg, cam_id))
            self.camera_subs.append(sub)
            
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.is_recording = False

    def camera_callback(self, msg, camera_id):
        try:
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape(msg.height, msg.width, -1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            q_img = QImage(image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            pixmap = QPixmap.fromImage(q_img)
            scaled_pixmap = pixmap.scaled(320, 240, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.camera_frames[camera_id].setPixmap(scaled_pixmap)
            
        except Exception as e:
            rospy.logerr("Error processing camera {} image: {}".format(camera_id, str(e)))

    def gps_callback(self, msg):
        try:
            gps_text = "Location: {:.6f}N, {:.6f}E".format(msg.latitude, msg.longitude)
            self.gps_label.setText(gps_text)
        except Exception as e:
            rospy.logerr("Error processing GPS data: {}".format(str(e)))

    def toggle_recording(self):
        try:
            rospy.wait_for_service('/recorder/set_recording', timeout=1.0)
            record_srv = rospy.ServiceProxy('/recorder/set_recording', SetBool)
            
            self.is_recording = not self.is_recording
            response = record_srv(self.is_recording)
            
            if response.success:
                self.record_button.setText("Stop Recording" if self.is_recording else "Start Recording")
                self.record_button.setProperty("recording", self.is_recording)
                self.record_button.setStyle(self.record_button.style())
                rospy.loginfo(response.message)
            else:
                rospy.logerr("Failed to toggle recording: {}".format(response.message))
                
        except Exception as e:
            rospy.logerr("Error toggling recording: {}".format(str(e)))

    def shutdown_plugin(self):
        for sub in self.camera_subs:
            sub.unregister()
        self.gps_sub.unregister()
