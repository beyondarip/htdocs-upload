#!/usr/bin/env python3


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import threading
import os
import torch
import torch.nn as nn
import torchvision.transforms as T
from PIL import Image as PILImage
from PIL import ImageDraw
import sys
import time

# Add RT-DETR path
sys.path.append('/home/ris/gps_camera_ws/ML/ML/rtdetrv2')
from src.core import YAMLConfig

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)

        # FPS calculation
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0

        # RT-DETR setup
        self.setup_model()

        # Parameter dasar
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)

        # Parameter untuk video lokal
        self.use_local_video = rospy.get_param('~use_local_video', False)
        self.video_path = rospy.get_param('~video_path', '')
        self.loop_video = rospy.get_param('~loop_video', True)

        self.image_pub = rospy.Publisher(
            f'/camera{self.camera_id}/image_raw',
            Image,
            queue_size=10
        )

        self.cap = None
        self.running = False
        self.lock = threading.Lock()

        # Model transforms
        self.transforms = T.Compose([
            T.Resize((640, 640)),
            T.ToTensor(),
        ])

        # Class mapping for detection
        self.class_mapping = {
            0: "AC", 1: "BC", 2: "BL", 3: "BS", 4: "CR", 5: "DP", 6: "EC",
            7: "JC", 8: "LS", 9: "LT", 10: "PA", 11: "PC", 12: "PH", 13: "RT",
            14: "SC", 15: "SL", 16: "SV", 17: "WR"
        }

    def setup_model(self):
        """Initialize RT-DETR model"""
        try:
            config_path = "/home/ris/gps_camera_ws/ML/ML/rtdetrv2/configs/rtdetrv2/rtdetrv2_r18vd_120e_coco.yml"
            model_path = "/home/ris/gps_camera_ws/src/gps_camera_monitoring/config/checkpoint0119.pth"
            self.device = rospy.get_param('~device', 'cpu')

            # Load configuration
            cfg = YAMLConfig(config_path, resume=model_path)
            checkpoint = torch.load(model_path, map_location='cpu')
            state = checkpoint['ema']['module'] if 'ema' in checkpoint else checkpoint['model']
            cfg.model.load_state_dict(state)

            # Setup model
            class Model(nn.Module):
                def __init__(self, cfg):
                    super().__init__()
                    self.model = cfg.model.deploy()
                    self.postprocessor = cfg.postprocessor.deploy()

                def forward(self, images, orig_target_sizes):
                    outputs = self.model(images)
                    outputs = self.postprocessor(outputs, orig_target_sizes)
                    return outputs

            self.model = Model(cfg).to(self.device)
            self.model.eval()
            rospy.loginfo("RT-DETR model loaded successfully")

        except Exception as e:
            rospy.logerr(f"Failed to load RT-DETR model: {str(e)}")
            raise

    def numpy_to_image_msg(self, frame):
        """Convert numpy array ke ROS image message"""
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

    def process_frame(self, frame):
        """Process frame through RT-DETR model"""
        try:
            # Calculate FPS
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            if elapsed_time > 1.0:
                self.fps = self.frame_count / elapsed_time
                self.frame_count = 0
                self.start_time = current_time

            # Convert to PIL
            pil_image = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            w, h = pil_image.size
            orig_size = torch.tensor([w, h])[None].to(self.device)

            # Transform and predict
            img_tensor = self.transforms(pil_image)[None].to(self.device)

            with torch.no_grad():
                labels, boxes, scores = self.model(img_tensor, orig_size)

            # Draw predictions
            self.draw_predictions(pil_image, labels[0], boxes[0], scores[0])

            # Convert back to OpenCV and add FPS
            processed_frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            cv2.putText(
                processed_frame,
                f"FPS: {self.fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )
            return processed_frame

        except Exception as e:
            rospy.logerr(f"Error in frame processing: {str(e)}")
            return frame

    def draw_predictions(self, image, labels, boxes, scores, threshold=0.6):
        """Draw predictions on image"""
        draw = ImageDraw.Draw(image)

        valid_mask = scores > threshold
        valid_boxes = boxes[valid_mask]
        valid_labels = labels[valid_mask]
        valid_scores = scores[valid_mask]

        for box, label, score in zip(valid_boxes, valid_labels, valid_scores):
            # Draw bounding box
            draw.rectangle(list(box), outline='red', width=2)

            # Get class name
            class_id = label.item()
            class_name = self.class_mapping.get(class_id, "Unknown")

            # Draw label and score
            label_text = f"{class_name}: {score.item():.2f}"
            draw.text((box[0], box[1] - 10), label_text, fill='red')

    def capture_and_publish(self):
        while not rospy.is_shutdown() and self.running:
            try:
                with self.lock:
                    if self.cap is None or not self.cap.isOpened():
                        if not self.connect_camera():
                            rospy.sleep(1)
                            continue

                    ret, frame = self.cap.read()

                    if self.use_local_video and not ret and self.loop_video:
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
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

                    # Process frame through RT-DETR
                    processed_frame = self.process_frame(frame)

                    # Resize frame if needed
                    if processed_frame.shape[1] != self.width or processed_frame.shape[0] != self.height:
                        processed_frame = cv2.resize(processed_frame, (self.width, self.height))

                    # Convert and publish
                    img_msg = self.numpy_to_image_msg(processed_frame)
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
