#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, NavSatFix
from std_srvs.srv import SetBool, SetBoolResponse
import os
from datetime import datetime
import threading

class RecorderNode:
    def __init__(self):
        rospy.init_node('recorder_node', anonymous=True)
        
        # Setup paths
        self.save_path = rospy.get_param('~save_path', '~/recorded_data')
        self.save_path = os.path.expanduser(self.save_path)
        os.makedirs(self.save_path, exist_ok=True)


        self.record_interval = rospy.get_param('/recorder/record_interval',1.0)
        rospy.loginfo(f"Recording interval set to {self.record_interval} secondss")
        
        # Status recording
        self.is_recording = False
        self.current_session = None
        self.lock = threading.Lock()
        
        # Latest data holders untuk 3 kamera
        self.latest_gps = None
        self.latest_images = {
            0: None,  # camera0
            1: None,  # camera1
            2: None   # camera2
        }
        
        # Subscribe ke GPS
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        # Subscribe ke 3 kamera
        rospy.Subscriber('/camera0/image_raw', Image, 
                        lambda msg: self.camera_callback(msg, 0))
        rospy.Subscriber('/camera1/image_raw', Image, 
                        lambda msg: self.camera_callback(msg, 1))
        rospy.Subscriber('/camera2/image_raw', Image, 
                        lambda msg: self.camera_callback(msg, 2))
        
        # Timer untuk recording periodic
        self.record_timer = None
        
        # Service untuk control recording
        self.record_service = rospy.Service(
            'recorder/set_recording',
            SetBool,
            self.handle_recording_request
        )
        
        rospy.loginfo("Recorder node initialized with 3 cameras")

    def create_session(self):
        """Buat session recording baru"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_path = os.path.join(self.save_path, timestamp)
        os.makedirs(session_path, exist_ok=True)
        os.makedirs(os.path.join(session_path, "images"), exist_ok=True)
        
        data_file = open(os.path.join(session_path, "gps_data.csv"), "w")
        # Header untuk 3 kamera
        data_file.write("timestamp,latitude,longitude,camera0_image,camera1_image,camera2_image\n")
        
        rospy.loginfo(f"Created new recording session at: {session_path}")
        return {
            'path': session_path,
            'data_file': data_file,
            'timestamp': timestamp
        }

    def gps_callback(self, msg):
        """Handle GPS data"""
        self.latest_gps = msg
        # rospy.loginfo("GPS data received")
        
    def camera_callback(self, msg, camera_id):
        """Handle camera data dengan ID"""
        self.latest_images[camera_id] = msg
        rospy.loginfo(f"Camera {camera_id} data received")

    def record_data(self, event=None):
        """Record data dari semua kamera"""
        if not self.is_recording:
            return
            
        try:
            with self.lock:
                if self.latest_gps is None:
                    rospy.logwarn("Missing GPS data")
                    # return
                    # jika gps tidak , maka jangan ambil gambar
                    
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                image_filenames = {}
                
                # Process setiap kamera
                for camera_id in range(3):
                    if self.latest_images[camera_id] is not None:
                        try:
                            # Convert image
                            image = np.frombuffer(self.latest_images[camera_id].data, 
                                                dtype=np.uint8)
                            image = image.reshape(
                                self.latest_images[camera_id].height,
                                self.latest_images[camera_id].width,
                                -1
                            )
                            
                            # Tambahkan label kamera pada gambar
                            #cv2.putText(image, f"Camera {camera_id}", (10, 30), 
                            #          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            
                            # Save image dengan nama yang jelas
                            image_filename = f"cam{camera_id}_{timestamp}.jpg"
                            image_path = os.path.join(
                                self.current_session['path'],
                                "images",
                                image_filename
                            )
                            cv2.imwrite(image_path, image)
                            image_filenames[camera_id] = image_filename
                            rospy.loginfo(f"Saved image from camera {camera_id}: {image_filename}")
                            
                        except Exception as e:
                            rospy.logerr(f"Error saving image from camera {camera_id}: {str(e)}")
                            image_filenames[camera_id] = "NA"
                    else:
                        image_filenames[camera_id] = "NA"
                
                # Write ke CSV dengan data semua kamera
                data_line = (
                    f"{timestamp},"
                    f"{self.latest_gps.latitude:.6f},"
                    f"{self.latest_gps.longitude:.6f},"
                    f"{image_filenames[0]},"
                    f"{image_filenames[1]},"
                    f"{image_filenames[2]}\n"
                )
                self.current_session['data_file'].write(data_line)
                self.current_session['data_file'].flush()
                rospy.loginfo("Saved data to CSV")
                    
        except Exception as e:
            rospy.logerr(f"Error in record_data: {str(e)}")

    def handle_recording_request(self, req):
        """Handle service request untuk start/stop recording"""
        try:
            if req.data and not self.is_recording:
                rospy.loginfo("Starting new recording session...")
                self.current_session = self.create_session()
                self.is_recording = True
                
                # Start periodic recording - setiap 1 detik
                self.record_timer = rospy.Timer(rospy.Duration(self.record_interval), self.record_data)
                
                msg = f"Recording started: {self.current_session['timestamp']}"
                rospy.loginfo(msg)
                return SetBoolResponse(True, msg)
                
            elif not req.data and self.is_recording:
                self.is_recording = False
                if self.record_timer:
                    self.record_timer.shutdown()
                if self.current_session:
                    self.current_session['data_file'].close()
                    timestamp = self.current_session['timestamp']
                    self.current_session = None
                    msg = f"Recording stopped: {timestamp}"
                    rospy.loginfo(msg)
                    return SetBoolResponse(True, msg)
                    
            return SetBoolResponse(True, "Recording already in requested state")
            
        except Exception as e:
            error_msg = f"Error handling recording request: {str(e)}"
            rospy.logerr(error_msg)
            return SetBoolResponse(False, error_msg)
            
    def run(self):
        """Main run loop"""
        try:
            rospy.loginfo("Recorder node is running...")
            rospy.spin()
        except Exception as e:
            rospy.logerr(f"Error in recorder node: {str(e)}")
        finally:
            if self.is_recording and self.current_session:
                self.current_session['data_file'].close()
                rospy.loginfo("Cleaned up recording session")

if __name__ == '__main__':
    try:
        recorder = RecorderNode()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
