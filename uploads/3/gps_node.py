#!/usr/bin/env python3

import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import threading

class GPSNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)
        
        # Ambil parameter dari ROS parameter server
        self.port = rospy.get_param('~port', '/dev/ttyTHS1')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        
        # Initialize publishers
        self.gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.status_pub = rospy.Publisher('gps/status', NavSatStatus, queue_size=10)
        
        self.serial_port = None
        self.running = False
        self.lock = threading.Lock()
        
    def connect(self):
        """Menghubungkan ke GPS device"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            rospy.loginfo(f"Successfully connected to GPS on port {self.port}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to connect to GPS: {str(e)}")
            return False
            
    def read_gps(self):
        """Membaca data GPS secara continuous"""
        while not rospy.is_shutdown() and self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8')
                    
                    if line.startswith('$GPGGA'):
                        msg = pynmea2.parse(line)
                        self.publish_gps_data(msg)
                        
            except Exception as e:
                rospy.logerr(f"Error reading GPS data: {str(e)}")
                rospy.sleep(1)
                
    def publish_gps_data(self, nmea_msg):
        """Publish data GPS ke ROS topics"""
        try:
            # Buat message NavSatFix
            fix_msg = NavSatFix()
            fix_msg.header = Header()
            fix_msg.header.stamp = rospy.Time.now()
            fix_msg.header.frame_id = "gps"
            
            # Set GPS fix data
            fix_msg.latitude = nmea_msg.latitude if nmea_msg.latitude else 0.0
            fix_msg.longitude = nmea_msg.longitude if nmea_msg.longitude else 0.0
            fix_msg.altitude = float(nmea_msg.altitude) if nmea_msg.altitude else 0.0
            
            # Set covariance (gunakan nilai default untuk saat ini)
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
            # Set status
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.status.status = NavSatStatus.STATUS_FIX if nmea_msg.gps_qual > 0 else NavSatStatus.STATUS_NO_FIX
            
            # Publish messages
            self.gps_pub.publish(fix_msg)
            self.status_pub.publish(fix_msg.status)
            
        except Exception as e:
            rospy.logerr(f"Error publishing GPS data: {str(e)}")
            
    def start(self):
        """Mulai node GPS"""
        if self.connect():
            self.running = True
            # Mulai thread untuk membaca GPS
            self.read_thread = threading.Thread(target=self.read_gps)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            rospy.loginfo("GPS Node started successfully")
            
            # Keep the node running
            rospy.spin()
        else:
            rospy.logerr("Failed to start GPS Node")
            
    def stop(self):
        """Stop node GPS dengan aman"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
        rospy.loginfo("GPS Node stopped")

if __name__ == '__main__':
    try:
        gps_node = GPSNode()
        gps_node.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'gps_node' in locals():
            gps_node.stop()
