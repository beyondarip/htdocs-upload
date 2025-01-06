import sys
import cv2
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                           QWidget, QGridLayout, QLabel, QVBoxLayout,
                           QMessageBox, QFrame, QProgressBar, QHBoxLayout,
                           QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QSize
from PyQt5.QtGui import QImage, QPixmap, QFont, QIcon, QPalette, QColor
import logging
from datetime import datetime
import os
from pathlib import Path
from PyQt5.QtCore import QThread, pyqtSignal
import cv2
import time

from datetime import datetime
import os
from pathlib import Path
import random

class GPSHandler:
    """Handle real GPS data from NEO-7M"""
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.current_data = {
            'latitude': 0,
            'longitude': 0,
            'satellites': 0,
            'quality': 0,
            'timestamp': ''
        }
        
    def start(self):
        """Start GPS reading"""
        try:
            import serial
            import pynmea2
            
            # Inisialisasi port serial untuk NEO-7M
            self.serial_port = serial.Serial(
                port='/dev/ttyTHS1',  # Port untuk Jetson, sesuaikan jika berbeda
                baudrate=9600,
                timeout=1
            )
            
            self.running = True
            
            # Start reading in thread
            self.read_thread = threading.Thread(target=self._read_gps, daemon=True)
            self.read_thread.start()
            
            return True
            
        except Exception as e:
            logger.log_error(e, "GPS initialization")
            return False
            
    def _read_gps(self):
        """Read GPS data continuously"""
        import pynmea2
        
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8')
                    
                    if line.startswith('$GPGGA'):
                        msg = pynmea2.parse(line)
                        
                        with threading.Lock():
                            self.current_data = {
                                'latitude': msg.latitude,
                                'longitude': msg.longitude,
                                'satellites': msg.num_sats,
                                'quality': msg.gps_qual,
                                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
                            }
                            
            except Exception as e:
                logger.log_error(e, "GPS reading")
                time.sleep(1)
                
    def get_coordinates(self):
        """Get current GPS coordinates"""
        with threading.Lock():
            return {
                'latitude': self.current_data['latitude'],
                'longitude': self.current_data['longitude']
            }
                
    def stop(self):
        """Stop GPS reading"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()

class DataRecorder:
    """Handle recording of GPS data and images"""
    def __init__(self):
        self.base_path = Path("recorded_data")
        self.current_session = None
        self.data_file = None
        
    def start_session(self):
        """Start a new recording session"""
        # Create timestamp-based session folder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_session = self.base_path / timestamp
        
        # Create directory structure
        self.current_session.mkdir(parents=True, exist_ok=True)
        (self.current_session / "images").mkdir(exist_ok=True)
        
        # Initialize data file
        self.data_file = open(self.current_session / f"data_{timestamp}.txt", "w")
        headers = [
            "Record_Time",
            "Latitude",
            "Longitude",
            "Camera1_Image",
            "Camera2_Image",
            "Camera3_Image"
        ]
        self.data_file.write(",".join(headers) + "\n")
        return timestamp
        
    def save_data(self, gps_data, frames):
        """Save GPS data and images in one synchronized record"""
        if not self.current_session or not self.data_file:
            return False
            
        try:
            # Generate timestamp untuk record ini
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # List untuk menyimpan nama file gambar
            image_files = ["NA", "NA", "NA"]  # Default "NA" jika tidak ada gambar
            
            # Simpan gambar untuk setiap kamera yang aktif
            for camera_id, frame in frames.items():
                if frame is not None:
                    image_filename = f"img_cam{camera_id}_{timestamp}.jpg"
                    image_path = self.current_session / "images" / image_filename
                    cv2.imwrite(str(image_path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    image_files[camera_id] = image_filename
            
            # Format data record
            data_record = [
                timestamp,
                f"{gps_data['latitude']:.6f}",
                f"{gps_data['longitude']:.6f}",
                image_files[0],
                image_files[1],
                image_files[2]
            ]
            
            # Tulis ke file
            self.data_file.write(",".join(data_record) + "\n")
            self.data_file.flush()
            return True
            
        except Exception as e:
            logger.log_error(e, "Error saving synchronized data")
            return False
            
    def stop_session(self):
        """Stop current recording session"""
        if self.data_file:
            self.data_file.close()
            self.data_file = None
        self.current_session = None

class VideoWorker(QThread):
    """Worker thread untuk capture video"""
    frame_ready = pyqtSignal(object)
    error = pyqtSignal(str)
    connected = pyqtSignal()
    
    def __init__(self, video_source):
        super().__init__()
        self.video_source = video_source
        self.running = False
        self.capture = None
        self.fps = 30
        
    def create_capture(self):
        """Buat video capture berdasarkan platform dan jenis sumber"""
        try:
            import platform
            system = platform.system()
            
            # Jika menggunakan webcam/camera
            if isinstance(self.video_source, int):
                if system == 'Linux':  # Untuk Jetson/Linux
                    # Cek device kamera tersedia
                    device_path = f"/dev/video{self.video_source}"
                    if not os.path.exists(device_path):
                        raise Exception(f"Camera device not found: {device_path}")
                        
                    # Coba dengan v4l2
                    cap = cv2.VideoCapture(self.video_source)
                    if not cap.isOpened():
                        # Jika gagal, coba dengan gstreamer
                        gst_str = (
                            f"v4l2src device=/dev/video{self.video_source} ! "
                            "video/x-raw, width=640, height=480, format=YUY2 ! "
                            "videoconvert ! video/x-raw, format=BGR ! "
                            "appsink"
                        )
                        cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                        
                    if not cap.isOpened():
                        raise Exception("Failed to open camera with both v4l2 and gstreamer")
                        
                else:  # Untuk Windows
                    cap = cv2.VideoCapture(self.video_source, cv2.CAP_DSHOW)
                    
            # Jika menggunakan file video
            else:
                if not os.path.exists(self.video_source):
                    raise Exception(f"Video file not found: {self.video_source}")
                cap = cv2.VideoCapture(self.video_source)
                
            return cap
            
        except Exception as e:
            self.error.emit(str(e))
            return None
            
    def run(self):
        """Main worker loop"""
        try:
            # Inisialisasi capture
            self.capture = self.create_capture()
            if self.capture is None or not self.capture.isOpened():
                self.error.emit(f"Failed to open video source: {self.video_source}")
                return
                
            # Set properti video
            if isinstance(self.video_source, str):
                self.fps = self.capture.get(cv2.CAP_PROP_FPS)
                if self.fps <= 0:
                    self.fps = 30
            
            # Set buffer size untuk mengurangi latency
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
            frame_delay = 1.0 / self.fps
            self.connected.emit()
            self.running = True
            
            previous_time = time.time()
            
            while self.running:
                # Frame timing control
                current_time = time.time()
                elapsed = current_time - previous_time
                
                if elapsed < frame_delay:
                    sleep_time = frame_delay - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                
                # Baca frame
                ret, frame = self.capture.read()
                
                if not ret:
                    # Untuk file video, loop kembali ke awal
                    if isinstance(self.video_source, str):
                        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    else:
                        # Untuk camera, coba reconnect
                        self.error.emit("Failed to capture frame")
                        break
                
                # Process frame
                try:
                    # Convert BGR ke RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # Resize jika terlalu besar (opsional, untuk performa)
                    if frame.shape[1] > 1280:  # Jika width > 1280
                        frame = cv2.resize(frame, (1280, 720))
                        
                    self.frame_ready.emit(frame)
                    previous_time = time.time()
                    
                except Exception as e:
                    self.error.emit(f"Frame processing error: {str(e)}")
                    continue
                    
        except Exception as e:
            self.error.emit(str(e))
            
        finally:
            # Cleanup
            if self.capture is not None:
                self.capture.release()
            self.running = False
                
    def stop(self):
        """Stop worker thread dengan aman"""
        self.running = False
        # Tunggu thread selesai
        self.wait()
        # Cleanup
        if self.capture is not None:
            self.capture.release()
            self.capture = None

# Set up logging with structured error handling
class AppLogger:
    def __init__(self):
        self.log_dir = Path('logs')
        self.log_dir.mkdir(exist_ok=True)
        
        self.log_file = self.log_dir / f'app_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
            handlers=[
                logging.FileHandler(self.log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
    def log_error(self, error, context=""):
        self.logger.error(f"{context} - {str(error)}", exc_info=True)

logger = AppLogger()


class ModernStyle:
    """Modern UI styling with light theme"""
    # Color palette - Modern Light Theme
    COLORS = {
        'background': '#ffffff',
        'surface': '#f8fafc',  # Light gray background
        'primary': '#3b82f6',  # Modern blue
        'secondary': '#6366f1', # Indigo
        'success': '#10b981',  # Emerald green
        'error': '#ef4444',    # Modern red
        'warning': '#f59e0b',  # Amber
        'text': '#1e293b',     # Slate gray
        'text_dim': '#64748b', # Lighter text
        'border': '#e2e8f0',   # Light border
        'hover': '#dbeafe'     # Light blue hover
    }

    BUTTON_BLOCK = f"""
        QPushButton {{
            background-color: {COLORS['primary']};
            color: white;
            border: none;
            border-radius: 8px;
            padding: 10px;
            font-weight: bold;
            font-size: 13px;
            width: 100%;
            margin: 5px 0;
            text-align: center;
        }}
        QPushButton:hover {{
            background-color: {COLORS['secondary']};
        }}
        QPushButton:pressed {{
            background-color: {COLORS['primary']};
        }}
        QPushButton[active="true"] {{
            background-color: {COLORS['error']};
        }}
        QPushButton[active="true"]:hover {{
            background-color: #dc2626;
        }}
    """

    CAMERA_HEADER = f"""
        QLabel {{
            color: {COLORS['text']};
            font-size: 12px;
            padding: 2px 5px;
            background-color: {COLORS['surface']};
            border: 1px solid {COLORS['border']};
            border-radius: 4px;
            max-width: 100px;
        }}
    """
    
    MAIN_WINDOW = f"""
        QMainWindow {{
            background-color: {COLORS['background']};
        }}
        QWidget {{
            color: {COLORS['text']};
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
        }}
        QLabel {{
            font-size: 13px;
        }}
    """
    
    VIDEO_FRAME = f"""
        QFrame {{
            background-color: {COLORS['surface']};
            border: 1px solid {COLORS['border']};
            border-radius: 12px;
            padding: 12px;
        }}
        QFrame:hover {{
            border-color: {COLORS['primary']};
            background-color: {COLORS['hover']};
        }}
        QLabel {{
            color: {COLORS['text']};
            font-size: 13px;
        }}
        QProgressBar {{
            border: none;
            background-color: {COLORS['border']};
            height: 2px;
            border-radius: 1px;
        }}
        QProgressBar::chunk {{
            background-color: {COLORS['primary']};
            border-radius: 1px;
        }}
    """
    
    CONTROL_BUTTON = f"""
        QPushButton {{
            background-color: {COLORS['primary']};
            color: white;
            border: none;
            border-radius: 8px;
            padding: 8px 16px;
            font-weight: bold;
            font-size: 13px;
            min-width: 100px;
        }}
        QPushButton:hover {{
            background-color: {COLORS['secondary']};
        }}
        QPushButton:pressed {{
            background-color: {COLORS['primary']};
            transform: translateY(1px);
        }}
    """
    
    WINDOW_CONTROLS = f"""
        QPushButton {{
            background-color: transparent;
            border: none;
            padding: 5px;
            border-radius: 4px;
        }}
        QPushButton:hover {{
            background-color: {COLORS['hover']};
        }}
    """

    GPS_FRAME = f"""
        QFrame {{
            background-color: {COLORS['surface']};
            border: 1px solid {COLORS['border']};
            border-radius: 12px;
            padding: 16px;
            min-width: 200px;
        }}
        QFrame:hover {{
            border-color: {COLORS['primary']};
            background-color: {COLORS['hover']};
        }}
        QLabel {{
            color: {COLORS['text']};
            font-size: 13px;
            padding: 2px 0;
        }}
        QLabel[class="title"] {{
            font-size: 14px;
            font-weight: bold;
            color: {COLORS['primary']};
        }}
    """

    def set_placeholder(self):
        """Set modern placeholder for inactive camera"""
        source_type = "Camera" if isinstance(self.video_source, int) else "Video"
        source_name = f"{source_type} {self.camera_id + 1}"
        if isinstance(self.video_source, str):
            source_name = os.path.basename(self.video_source)
            
        placeholder_html = f"""
            <div style='
                text-align: center;
                padding: 20px;
                color: {ModernStyle.COLORS['text_dim']};
                background-color: {ModernStyle.COLORS['surface']};
                border-radius: 8px;
            '>
                <div style='font-size: 24px; margin-bottom: 10px;'>ðŸ“¹</div>
                <div style='font-weight: bold; margin-bottom: 5px; color: {ModernStyle.COLORS['text']};'>
                    {source_name}
                </div>
                <div>Waiting for video input...</div>
            </div>
        """
        self.video_frame.setText(placeholder_html)
        
    def start_video(self):
        """Start video capture with enhanced error handling"""
        try:
            self.progress_bar.show()
            self.status_label.setText("Connecting...")
            self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['warning']};")
            
            self.capture = cv2.VideoCapture(self.camera_id)
            if not self.capture.isOpened():
                raise Exception(f"Failed to connect to camera {self.camera_id + 1}")
            
            self.timer.start(33)  # ~30 fps
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['success']};")
            self.progress_bar.hide()
            
        except Exception as e:
            self.handle_video_error(str(e))
            
    def handle_video_error(self, error_msg):
        """Enhanced error handling for video issues"""
        self.error_count += 1
        logger.log_error(error_msg, f"Camera {self.camera_id + 1}")
        
        self.progress_bar.hide()
        self.status_label.setText("Error")
        self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['error']};")
        self.set_placeholder()
        
        if self.error_count >= self.max_error_retries:
            QMessageBox.warning(
                self,
                "Camera Error",
                f"Failed to connect to Camera {self.camera_id + 1}\n{error_msg}",
                QMessageBox.Ok
            )
        
        self.error_timer.start(5000)  # Reset error count after 5 seconds

class VideoDisplay(QFrame):
    """Enhanced video display widget with modern UI"""
    def __init__(self, camera_id=0, parent=None):
        super().__init__(parent)
        self.camera_id = camera_id

        # PILIH
        self.video_source = self.camera_id  # Untuk webcam
        # self.video_source = f"./qqy.mp4"# Untuk file video lokal

        self.setup_ui()
        self.setup_error_handling()
        self.worker = None
        self.current_frame = None  # Unt
        
    def setup_ui(self):
        """Initialize the UI components with modern styling"""
        self.setStyleSheet(ModernStyle.VIDEO_FRAME)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Main layout with minimal spacing
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Compact header
        header_layout = QHBoxLayout()
        header_layout.setSpacing(5)
        
        self.title_label = QLabel(f"Camera {self.camera_id + 1}")
        self.title_label.setStyleSheet(ModernStyle.CAMERA_HEADER)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet(ModernStyle.CAMERA_HEADER)
        
        header_layout.addWidget(self.title_label)
        header_layout.addWidget(self.status_label)
        header_layout.addStretch()
        layout.addLayout(header_layout)
        
        # Video display area
        self.video_frame = QLabel()
        self.video_frame.setAlignment(Qt.AlignCenter)
        self.video_frame.setMinimumSize(280, 210)
        layout.addWidget(self.video_frame)
        
        # Progress indicator
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setMaximumHeight(2)
        self.progress_bar.hide()
        layout.addWidget(self.progress_bar)
        
        # Set default placeholder
        self.set_placeholder()
        
    def setup_error_handling(self):
        """Set up error handling for the video stream"""
        self.error_count = 0
        self.max_error_retries = 3
        self.error_timer = QTimer()
        self.error_timer.timeout.connect(self.reset_error_count)
        
    def reset_error_count(self):
        """Reset error counter after timeout"""
        self.error_count = 0
        
    def set_placeholder(self):
        """Set modern placeholder for inactive camera"""
        source_type = "Camera" if isinstance(self.video_source, int) else "Video"
        source_name = f"{source_type} {self.camera_id + 1}"
        if isinstance(self.video_source, str):
            source_name = os.path.basename(self.video_source)
            
        placeholder_html = f"""
            <div style='
                text-align: center;
                padding: 20px;
                color: {ModernStyle.COLORS['text_dim']};
                background-color: {ModernStyle.COLORS['surface']};
                border-radius: 8px;
            '>
                <div style='font-size: 24px; margin-bottom: 10px;'>ðŸ“¹</div>
                <div style='font-weight: bold; margin-bottom: 5px; color: {ModernStyle.COLORS['text']};'>
                    {source_name}
                </div>
                <div>Waiting for video input...</div>
            </div>
        """
        self.video_frame.setText(placeholder_html)
        
    def start_video(self):
        """Start video capture in a separate thread"""
        try:
            self.progress_bar.show()
            self.status_label.setText("Connecting...")
            self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['warning']};")
            
            # Create and start worker thread
            self.worker = VideoWorker(self.video_source)
            self.worker.frame_ready.connect(self.update_frame)
            self.worker.error.connect(self.handle_video_error)
            self.worker.connected.connect(self.handle_connected)
            self.worker.start()
            
        except Exception as e:
            self.handle_video_error(str(e))
            
    def stop_video(self):
        """Safely stop video capture"""
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.set_placeholder()
        self.status_label.setText("Stopped")
        self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['text_dim']};")
    
    def handle_connected(self):
        """Handle successful connection"""
        self.status_label.setText("Connected")
        self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['success']};")
        self.progress_bar.hide()
            
    def handle_video_error(self, error_msg):
        """Enhanced error handling for video issues"""
        self.error_count += 1
        logger.log_error(error_msg, f"Camera {self.camera_id + 1}")
        
        self.progress_bar.hide()
        self.status_label.setText("Error")
        self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['error']};")
        self.set_placeholder()
        
        if self.error_count >= self.max_error_retries:
            QMessageBox.warning(
                self,
                "Camera Error",
                f"Failed to connect to Camera {self.camera_id + 1}\n{error_msg}",
                QMessageBox.Ok
            )
        
        self.error_timer.start(5000)  # Reset error count after 5 seconds
        
    def update_frame(self, frame):
        """Update the video frame and store it"""
        try:
            # Store current frame for recording
            self.current_frame = frame.copy()  # Simpan copy frame untuk recording
            
            # Update display
            h, w, ch = frame.shape
            qt_image = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.video_frame.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_frame.setPixmap(scaled_pixmap)
            
        except Exception as e:
            self.handle_video_error(str(e))
            
    def closeEvent(self, event):
        """Handle widget closing"""
        self.stop_video()
        super().closeEvent(event)

class GPSDisplay(QFrame):
    """Modern GPS information display"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
    def setup_ui(self):
        """Initialize modern GPS display UI"""
        self.setStyleSheet(ModernStyle.VIDEO_FRAME)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # Title
        title = QLabel("GPS Location")
        title.setStyleSheet(f"color: {ModernStyle.COLORS['primary']}; font-weight: bold;")
        layout.addWidget(title)
        
        # Coordinates
        self.lat_label = QLabel("Latitude: --Â°")
        self.lon_label = QLabel("Longitude: --Â°")
        layout.addWidget(self.lat_label)
        layout.addWidget(self.lon_label)
        
        # Status
        self.status_label = QLabel("Searching for signal...")
        self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['warning']};")
        layout.addWidget(self.status_label)
        
    def update_location(self, lat=None, lon=None):
        """Update GPS information"""
        if lat is not None and lon is not None:
            self.lat_label.setText(f"Latitude: {lat:.6f}Â°")
            self.lon_label.setText(f"Longitude: {lon:.6f}Â°")
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['success']};")
        else:
            self.status_label.setText("Searching...")
            self.status_label.setStyleSheet(f"color: {ModernStyle.COLORS['warning']};")

class MonitoringSystem(QMainWindow):
    """Main application window with modern UI"""
    def __init__(self):
        super().__init__()
        self.is_recording = False
        self.is_sending = False
        self.setup_window()
        self.setup_ui()
        self.setup_timers()
        # Auto-start cameras
        # self.start_all_cameras()

           # Initialize GPS
        self.gps_handler = GPSHandler()
        self.gps_handler.start()  # Start GPS reading
        

        self.recorder = DataRecorder()
        self.recording_timer = QTimer()
        self.recording_timer.timeout.connect(self.record_data)
        
    def setup_window(self):
        """Configure main window properties"""
        self.setWindowTitle("GPS Camera Monitoring")
        self.setStyleSheet(ModernStyle.MAIN_WINDOW)
        screen = QApplication.primaryScreen().geometry()
        self.resize(int(screen.width() * 0.8), int(screen.height() * 0.8))
        self.center_window()
        

    def record_data(self):
        """Record one set of synchronized data"""
        try:
            # Collect frames from active cameras
            frames = {}
            for i, display in enumerate(self.video_displays):
                if display.worker and display.worker.running and hasattr(display, 'current_frame'):
                    if display.current_frame is not None:
                        frames[i] = display.current_frame
            
            # Skip if no active cameras
            if not frames:
                return
            
            # Get GPS data (dummy data untuk contoh)
            # gps_data = {
            #     'latitude': -6.2088 + random.uniform(-0.01, 0.01),
            #     'longitude': 106.8456 + random.uniform(-0.01, 0.01)
            # }
            gps_data = self.gps_handler.get_coordinates()
            
            # Save synchronized data
            self.recorder.save_data(gps_data, frames)
            
        except Exception as e:
            logger.log_error(e, "Data recording error")

    def start_all_cameras(self):
        """Start all cameras automatically"""
        for display in self.video_displays:
            display.start_video()

    def toggle_cameras(self):
        """Toggle camera state"""
        try:
            is_active = self.camera_button.property("active")
            if is_active:
                self.camera_button.setText("Nyalakan Camera")
                self.camera_button.setProperty("active", False)
                self.camera_button.setStyle(self.camera_button.style())
                for display in self.video_displays:
                    display.stop_video()
            else:
                self.camera_button.setText("Matikan Camera")
                self.camera_button.setProperty("active", True)
                self.camera_button.setStyle(self.camera_button.style())
                for display in self.video_displays:
                    display.start_video()
        except Exception as e:
            logger.log_error(e, "Camera toggle")
            QMessageBox.warning(self, "Error", f"Error toggling cameras: {str(e)}")


    def toggle_recording(self):
        """Toggle data recording"""
        try:
            self.is_recording = not self.is_recording
            if self.is_recording:
                # Start new recording session
                session_id = self.recorder.start_session()
                self.record_button.setText("Stop Take Data")
                self.record_button.setProperty("active", True)
                self.recording_timer.start(1000)  # Record every second
                # logger.info(f"Started recording session: {session_id}")
                
                QMessageBox.information(
                    self,
                    "Recording Started",
                    f"Recording data to folder: {session_id}",
                    QMessageBox.Ok
                )
            else:
                # Stop recording
                self.recording_timer.stop()
                self.recorder.stop_session()
                self.record_button.setText("Start Take Data")
                self.record_button.setProperty("active", False)
                # logger.info("Stopped recording session")
                
            self.record_button.setStyle(self.record_button.style())
            
        except Exception as e:
            logger.log_error(e, "Recording toggle")
            QMessageBox.warning(
                self,
                "Error",
                f"Error toggling recording: {str(e)}",
                QMessageBox.Ok
            )

    def toggle_sending(self):
        """Toggle data sending"""
        try:
            self.is_sending = not self.is_sending
            if self.is_sending:
                self.send_button.setText("Stop Kirim Data")
                self.send_button.setProperty("active", True)
                # Start sending logic here
            else:
                self.send_button.setText("Start Kirim Data")
                self.send_button.setProperty("active", False)
                # Stop sending logic here
            self.send_button.setStyle(self.send_button.style())
        except Exception as e:
            logger.log_error(e, "Sending toggle")
            QMessageBox.warning(self, "Error", f"Error toggling data sending: {str(e)}")

    def center_window(self):
        """Center window on screen"""
        frame = self.frameGeometry()
        center = QApplication.primaryScreen().availableGeometry().center()
        frame.moveCenter(center)
        self.move(frame.topLeft())


    def setup_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)

        # Top section: GPS and Controls
        top_section = QHBoxLayout()
        
        # Left side: GPS Display
        self.gps_display = GPSDisplay()
        top_section.addWidget(self.gps_display)

        # Center: Control Buttons
        control_layout = QVBoxLayout()
        control_layout.setSpacing(5)

        # Camera control button
        self.camera_button = QPushButton("Nyalakan Kamera")
        self.camera_button.setProperty("active", False)
        self.camera_button.setStyleSheet(ModernStyle.BUTTON_BLOCK)
        self.camera_button.clicked.connect(self.toggle_cameras)
        control_layout.addWidget(self.camera_button)

        # Record data button
        self.record_button = QPushButton("Start Take Data")
        self.record_button.setStyleSheet(ModernStyle.BUTTON_BLOCK)
        self.record_button.clicked.connect(self.toggle_recording)
        control_layout.addWidget(self.record_button)

        top_section.addLayout(control_layout)
        top_section.setStretchFactor(control_layout, 1)
        main_layout.addLayout(top_section)

        # Video section with send data button
        video_section = QHBoxLayout()
        
        # Video grid
        video_layout = QGridLayout()
        video_layout.setSpacing(10)
        
        self.video_displays = []
        for i in range(3):
            display = VideoDisplay(camera_id=i)
            self.video_displays.append(display)
            video_layout.addWidget(display, 0, i)
        
        video_section.addLayout(video_layout)
        
        # Send data control
        send_control = QVBoxLayout()
        self.send_button = QPushButton("Start Kirim Data")
        self.send_button.setStyleSheet(ModernStyle.BUTTON_BLOCK)
        self.send_button.clicked.connect(self.toggle_sending)
        send_control.addWidget(self.send_button)
        send_control.addStretch()
        
        video_section.addLayout(send_control)
        main_layout.addLayout(video_section)


    def setup_timers(self):
        """Initialize application timers"""
        # GPS update timer
        self.gps_timer = QTimer()
        self.gps_timer.timeout.connect(self.update_gps)
        self.gps_timer.start(1000)
        
    @pyqtSlot()
    def toggle_scan(self):
        """Toggle video scanning with error handling"""
        try:
            sender = self.sender()
            if sender.text() == "Start Scan":
                sender.setText("Stop Scan")
                sender.setStyleSheet(ModernStyle.CONTROL_BUTTON.replace(
                    ModernStyle.COLORS['primary'], 
                    ModernStyle.COLORS['error']
                ))
                for display in self.video_displays:
                    display.start_video()
            else:
                sender.setText("Start Scan")
                sender.setStyleSheet(ModernStyle.CONTROL_BUTTON)
                for display in self.video_displays:
                    display.stop_video()
                    
        except Exception as e:
            logger.log_error(e, "Scan toggle")
            QMessageBox.warning(
                self,
                "Operation Error",
                f"Error during scan operation:\n{str(e)}",
                QMessageBox.Ok
            )
            
    def update_gps(self):
        """Update GPS information (dummy implementation)"""
        try:
            # # TODO: Implement actual GPS
            # import random
            # if random.random() < 0.8:  # 80% chance of success
            #     lat = -6.2088 + random.uniform(-0.01, 0.01)  # Jakarta area
            #     lon = 106.8456 + random.uniform(-0.01, 0.01)
            #     self.gps_display.update_location(lat, lon)
            # else:
            #     self.gps_display.update_location()

            gps_data = self.gps_handler.get_coordinates()
            if gps_data['latitude'] != 0 and gps_data['longitude'] != 0:
                self.gps_display.update_location(
                    gps_data['latitude'],
                    gps_data['longitude']
                )
            else:
                self.gps_display.update_location()
        except Exception as e:
            logger.log_error(e, "GPS update")
            
    def closeEvent(self, event):
        """Handle application closing"""
        try:
            # Stop all video captures
            for display in self.video_displays:
                display.stop_video()
            
            # Stop timers
            self.gps_timer.stop()
            self.gps_handler.stop()
            
            # Accept the close event
            event.accept()
            
        except Exception as e:
            logger.log_error(e, "Application closing")
            event.accept()

def main():
    """Main application entry point with error handling"""
    try:
        app = QApplication(sys.argv)
        
        # Set application-wide style defaults
        app.setStyle('Fusion')
        
        # Create and show main window
        window = MonitoringSystem()
        window.show()
        
        # Start application event loop
        sys.exit(app.exec_())
        
    except Exception as e:
        logger.log_error(e, "Application startup")
        QMessageBox.critical(
            None,
            "Critical Error",
            f"Failed to start application:\n{str(e)}",
            QMessageBox.Ok
        )
        sys.exit(1)

if __name__ == '__main__':
    main()
