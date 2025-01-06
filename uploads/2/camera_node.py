#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import threading
import os

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        
        # Parameter dasar
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        
        # Parameter baru untuk video lokal
        self.use_local_video = rospy.get_param('~use_local_video', False)  # Flag untuk switch mode
        self.video_path = rospy.get_param('~video_path', '')  # Path ke video lokal
        self.loop_video = rospy.get_param('~loop_video', True)  # Option untuk loop video
        
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
            if self.use_local_video:
                if not os.path.exists(self.video_path):
                    raise Exception(f"Video file not found: {self.video_path}")
                self.cap = cv2.VideoCapture(self.video_path)
                rospy.loginfo(f"Connected to local video: {self.video_path}")
            else:
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
                raise Exception(f"Could not open {'video file' if self.use_local_video else f'camera {self.camera_id}'}")
                    
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            rospy.loginfo(f"{'Video' if self.use_local_video else f'Camera {self.camera_id}'} connected successfully")
            return True
                
        except Exception as e:
            rospy.logerr(f"Failed to connect to {'video' if self.use_local_video else f'camera {self.camera_id}'}: {str(e)}")
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
                    
                    # Handle video looping untuk local video
                    if self.use_local_video and not ret and self.loop_video:
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset video ke awal
                        ret, frame = self.cap.read()
                    
                    if not ret:
                        if self.use_local_video:
                            rospy.loginfo("End of video reached")
                            if not self.loop_video:
                                self.running = False
                            continue
                        else:
                            rospy.logwarn(f"Failed to capture frame from camera {self.camera_id}")
                            continue
                    
                    # Resize frame jika diperlukan
                    if frame.shape[1] != self.width or frame.shape[0] != self.height:
                        frame = cv2.resize(frame, (self.width, self.height))
                    
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
                
                rospy.loginfo(f"{'Video' if self.use_local_video else f'Camera'} node {self.camera_id} started successfully")
                rospy.spin()
            else:
                rospy.logerr(f"Failed to start {'video' if self.use_local_video else f'camera'} node {self.camera_id}")
                
        except Exception as e:
            rospy.logerr(f"Error starting {'video' if self.use_local_video else f'camera'} node: {str(e)}")
            
    def stop(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        if self.cap:
            self.cap.release()
        rospy.loginfo(f"{'Video' if self.use_local_video else f'Camera'} node {self.camera_id} stopped")

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        camera_node.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'camera_node' in locals():
            camera_node.stop()
