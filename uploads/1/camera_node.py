#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import threading

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        
        self.image_pub = rospy.Publisher(
            f'/camera{self.camera_id}/image_raw',
            Image,
            queue_size=10
        )
        
        self.cap = None
        self.running = False
        self.lock = threading.Lock()
        
    def numpy_to_image_msg(self, frame):
        """Convert numpy array ke ROS image message tanpa cv_bridge"""
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def connect_camera(self):
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            if not self.cap.isOpened():
                gst_str = (
                    f"v4l2src device=/dev/video{self.camera_id} ! "
                    f"video/x-raw, width={self.width}, height={self.height} ! "
                    "videoconvert ! video/x-raw, format=BGR ! "
                    "appsink"
                )
                self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                
            if not self.cap.isOpened():
                raise Exception(f"Could not open camera {self.camera_id}")
                
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            rospy.loginfo(f"Camera {self.camera_id} connected successfully")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to camera {self.camera_id}: {str(e)}")
            return False
            
    def capture_and_publish(self):
        while not rospy.is_shutdown() and self.running:
            try:
                with self.lock:
                    if self.cap is None or not self.cap.isOpened():
                        if not self.connect_camera():
                            rospy.sleep(1)
                            continue
                            
                    ret, frame = self.cap.read()
                    
                    if not ret:
                        rospy.logwarn(f"Failed to capture frame from camera {self.camera_id}")
                        continue
                    
                    # Convert dan publish
                    img_msg = self.numpy_to_image_msg(frame)
                    img_msg.header.stamp = rospy.Time.now()
                    img_msg.header.frame_id = f"camera_{self.camera_id}"
                    self.image_pub.publish(img_msg)
                        
            except Exception as e:
                rospy.logerr(f"Error in capture loop: {str(e)}")
                rospy.sleep(1)
                
            rate = rospy.Rate(self.frame_rate)
            rate.sleep()
            
    def start(self):
        try:
            if self.connect_camera():
                self.running = True
                self.capture_thread = threading.Thread(target=self.capture_and_publish)
                self.capture_thread.daemon = True
                self.capture_thread.start()
                
                rospy.loginfo(f"Camera node {self.camera_id} started successfully")
                rospy.spin()
            else:
                rospy.logerr(f"Failed to start camera node {self.camera_id}")
                
        except Exception as e:
            rospy.logerr(f"Error starting camera node: {str(e)}")
            
    def stop(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        if self.cap:
            self.cap.release()
        rospy.loginfo(f"Camera node {self.camera_id} stopped")

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        camera_node.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'camera_node' in locals():
            camera_node.stop()
