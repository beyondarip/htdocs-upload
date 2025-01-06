import sys
import cv2
import math
import time
import board
import busio
import logging
import os
import requests
import threading
import RPi.GPIO as GPIO
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("vending_machine.log"),
        logging.StreamHandler()
    ]
)

# GPIO Configuration
FLOW_SENSOR_PIN = 20
MOTOR_PIN = 21

# Calibrated pulses for different volumes
TARGET_PULSES = {
    "100 ml": 108,
    "350 ml": 380,
    "600 ml": 670,
    "1 Liter": 1080
}

class WaterController(QObject):
    update_progress = pyqtSignal(int)
    filling_complete = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.pulse_count = 0
        self.is_running = False
        self.target_pulses = 0
        self.setup_gpio()
        logging.info("Water Controller initialized")

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(FLOW_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(MOTOR_PIN, GPIO.OUT)
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            logging.info("GPIO setup completed successfully")
        except Exception as e:
            logging.error(f"Error setting up GPIO: {e}")
            raise

    def pulse_callback(self, channel):
        if not self.is_running:
            return

        self.pulse_count += 1
        progress = int((self.pulse_count / self.target_pulses) * 100)
        progress = min(100, progress)
        self.update_progress.emit(progress)
        logging.debug(f"Pulse count: {self.pulse_count}/{self.target_pulses}")

        if self.pulse_count >= self.target_pulses:
            self.stop_filling()

    def start_filling(self, size):
        if size not in TARGET_PULSES:
            logging.error(f"Invalid size selected: {size}")
            return False

        try:
            self.target_pulses = TARGET_PULSES[size]
            self.pulse_count = 0
            self.is_running = True
            
            # Start motor in separate thread
            threading.Thread(target=self._filling_process, daemon=True).start()
            logging.info(f"Starting filling process for {size}")
            return True
        except Exception as e:
            logging.error(f"Error starting filling process: {e}")
            return False

    def _filling_process(self):
        try:
            GPIO.output(MOTOR_PIN, GPIO.HIGH)
            GPIO.add_event_detect(FLOW_SENSOR_PIN, GPIO.FALLING, 
                                callback=self.pulse_callback)

            while self.is_running and self.pulse_count < self.target_pulses:
                time.sleep(0.1)

        except Exception as e:
            logging.error(f"Error in filling process: {e}")
        finally:
            self.stop_filling()

    def stop_filling(self):
        try:
            self.is_running = False
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            GPIO.remove_event_detect(FLOW_SENSOR_PIN)
            self.filling_complete.emit()
            logging.info(f"Filling completed. Pulses: {self.pulse_count}/{self.target_pulses}")
        except Exception as e:
            logging.error(f"Error stopping filling process: {e}")

    def cleanup(self):
        try:
            self.stop_filling()
            GPIO.cleanup()
            logging.info("GPIO cleanup completed")
        except Exception as e:
            logging.error(f"Error during GPIO cleanup: {e}")

class SensorThread(QThread):
    sensor_updated = pyqtSignal(dict)
    
    def __init__(self, esp32_ip):
        super().__init__()
        self.esp32_ip = esp32_ip
        self.running = True
        logging.info(f"Sensor thread initialized with ESP32 IP: {esp32_ip}")
        
    def run(self):
        while self.running:
            try:
                response = requests.get(f"http://{self.esp32_ip}/data", timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    self.sensor_updated.emit(data)
                    logging.debug(f"Received sensor data: {data}")
                else:
                    logging.warning(f"HTTP {response.status_code} from ESP32")
                    self.sensor_updated.emit({'error': True})
            except requests.exceptions.RequestException as e:
                logging.error(f"Error connecting to ESP32: {e}")
                self.sensor_updated.emit({'error': True})
            time.sleep(2)
            
    def stop(self):
        self.running = False
        logging.info("Sensor thread stopped")

class VideoThread(QThread):
    frame_ready = pyqtSignal(QImage)
    
    def __init__(self, video_path):
        super().__init__()
        self.video_path = video_path
        self.running = True
        self.mutex = QMutex()
        logging.info(f"Video thread initialized with path: {video_path}")
        
    def run(self):
        try:
            cap = cv2.VideoCapture(self.video_path)
            if not cap.isOpened():
                logging.error("Failed to open video file")
                return
                
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                
                self.mutex.lock()
                try:
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_frame.shape
                    bytes_per_line = ch * w
                    
                    image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                    self.frame_ready.emit(image.copy())
                finally:
                    self.mutex.unlock()
                    
                self.msleep(33)  # ~30 FPS
                
            cap.release()
            
        except Exception as e:
            logging.error(f"Error in video thread: {e}")
            
    def stop(self):
        self.mutex.lock()
        self.running = False
        self.mutex.unlock()
        logging.info("Video thread stopped")

class WaterButton(QPushButton):
    def __init__(self, size_text, image_path, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setFixedSize(150, 150)
        self.setCheckable(True)
        
        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(layout)
        
        # Icon container
        icon_container = QLabel()
        icon_container.setFixedSize(80, 80)
        icon_container.setStyleSheet("""
            background-color: white;
            border-radius: 10px;
            padding: 5px;
        """)
        
        # Load and set bottle image
        try:
            pixmap = QPixmap(image_path)
            scaled_pixmap = pixmap.scaled(70, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon_container.setPixmap(scaled_pixmap)
        except Exception as e:
            logging.error(f"Error loading image {image_path}: {e}")
            icon_container.setText("No Image")
            
        icon_container.setAlignment(Qt.AlignCenter)
        
        # Size label
        size_label = QLabel(size_text)
        size_label.setAlignment(Qt.AlignCenter)
        size_label.setStyleSheet("""
            color: #2C3E50;
            font-size: 16px;
            font-weight: bold;
            font-family: 'Segoe UI', Arial;
        """)
        
        layout.addWidget(icon_container, alignment=Qt.AlignCenter)
        layout.addWidget(size_label, alignment=Qt.AlignCenter)
        
        self.setStyleSheet("""
            WaterButton {
                background-color: #F8F9FA;
                border-radius: 15px;
                border: 2px solid #E9ECEF;
            }
            WaterButton:checked {
                background-color: #4EA8DE;
                border: 2px solid #5390D9;
            }
            WaterButton:checked QLabel {
                color: white;
            }
            WaterButton:hover {
                background-color: #E9ECEF;
                border: 2px solid #DEE2E6;
            }
        """)
class MachineWidget(QWidget):
    filling_completed = pyqtSignal()

    def __init__(self, water_controller):
        super().__init__()
        self.water_controller = water_controller
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(120)
        
        self.progress = 0
        self.is_filling = False
        self.progress_segments = 18
        
        self.setup_layout()
        logging.info("Machine Widget initialized")
        
    def setup_layout(self):
        layout = QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(layout)

        layout.addWidget(self.create_progress_container())
        layout.addWidget(self.create_machine_display())

    def create_progress_container(self):
        progress_container = QWidget()
        progress_container.setStyleSheet("""
            background-color: #2C3E50;
            border-radius: 15px;
            padding: 10px;
        """)
        
        progress_layout = QHBoxLayout(progress_container)
        progress_layout.setContentsMargins(10, 5, 10, 5)
        
        self.progress_indicator = QLabel("█")
        self.progress_indicator.setStyleSheet("color: #2ECC71; font-size: 24px;")
        
        self.progress_bar = QLabel("▬" * self.progress_segments)
        self.progress_bar.setStyleSheet("color: white; font-size: 16px;")
        
        progress_layout.addWidget(self.progress_indicator)
        progress_layout.addWidget(self.progress_bar, 1)
        
        return progress_container

    def create_machine_display(self):
        machine_display = QWidget()
        machine_display.setStyleSheet("""
            background-color: #2C3E50;
            border-radius: 15px;
            padding: 10px;
        """)
        
        machine_layout = QVBoxLayout(machine_display)
        machine_layout.setSpacing(5)
        machine_layout.setContentsMargins(10, 5, 10, 5)
        
        self.machine_image = QLabel()
        self.machine_image.setFixedSize(80, 80)
        self.machine_image.setAlignment(Qt.AlignCenter)
        self.machine_image.setStyleSheet("background: transparent;")
        
        try:
            if os.path.exists("5.png"):
                machine_pixmap = QPixmap("5.png")
                scaled_machine = machine_pixmap.scaled(70, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.machine_image.setPixmap(scaled_machine)
        except Exception as e:
            logging.error(f"Error loading machine image: {e}")
        
        self.start_button = QPushButton("Start Filling")
        self.start_button.setFixedSize(150, 40)
        self.start_button.setEnabled(False)
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #2ECC71;
                border-radius: 20px;
                color: white;
                font-size: 16px;
                font-weight: bold;
                font-family: 'Segoe UI', Arial;
                border: none;
            }
            QPushButton:disabled {
                background-color: #95A5A6;
            }
            QPushButton:hover:!disabled {
                background-color: #27AE60;
            }
        """)
        
        machine_layout.addWidget(self.machine_image, alignment=Qt.AlignCenter)
        machine_layout.addWidget(self.start_button, alignment=Qt.AlignCenter)
        
        return machine_display

    def update_progress(self, value):
        self.progress = value
        filled = "█" * int(self.progress_segments * value / 100)
        empty = "▬" * (self.progress_segments - int(self.progress_segments * value / 100))
        self.progress_bar.setText(filled + empty)

    def start_filling(self, size):
        if not self.is_filling:
            self.is_filling = True
            self.progress = 0
            self.start_button.setEnabled(False)
            self.start_button.setText("Filling...")
            self.progress_indicator.setStyleSheet("color: #E74C3C; font-size: 24px;")
            
            # Start the actual filling process
            success = self.water_controller.start_filling(size)
            if not success:
                self.complete_filling()

    def complete_filling(self):
        self.is_filling = False
        self.start_button.setEnabled(True)
        self.start_button.setText("Start Filling")
        self.progress_indicator.setStyleSheet("color: #2ECC71; font-size: 24px;")
        self.filling_completed.emit()
        logging.info("Filling process completed")

    def reset(self):
        if self.is_filling:
            self.water_controller.stop_filling()
        self.is_filling = False
        self.progress = 0
        self.progress_bar.setText("▬" * self.progress_segments)
        self.start_button.setEnabled(True)
        self.start_button.setText("Start Filling")
        self.progress_indicator.setStyleSheet("color: #2ECC71; font-size: 24px;")
        logging.info("Machine widget reset")

class MonitoringWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumHeight(150)
        
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(8, 8, 8, 8)
        self.setLayout(layout)
        
        self.esp32_ip = "192.168.137.117"  # Consider making this configurable
        logging.info(f"Initializing Monitoring Widget with ESP32 IP: {self.esp32_ip}")
        
        title = QLabel("Water Quality Monitoring")
        title.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: #2C3E50;
            font-family: 'Segoe UI', Arial;
        """)
        title.setAlignment(Qt.AlignCenter)
        
        monitor_container = QWidget()
        monitor_container.setStyleSheet("""
            background-color: #F8F9FA;
            border-radius: 15px;
            border: 2px solid #E9ECEF;
            padding: 10px;
        """)
        monitor_layout = QHBoxLayout(monitor_container)
        monitor_layout.setContentsMargins(8, 4, 8, 4)
        
        self.ph_widget = self.create_monitor_display("pH Value")
        self.tds_widget = self.create_monitor_display("TDS Value")
        
        monitor_layout.addWidget(self.ph_widget)
        monitor_layout.addWidget(self.tds_widget)
        
        layout.addWidget(title)
        layout.addWidget(monitor_container)
        
        self.sensor_thread = SensorThread(self.esp32_ip)
        self.sensor_thread.sensor_updated.connect(self.update_sensor_display)
        self.sensor_thread.start()

    def create_monitor_display(self, title):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(4)
        
        title_label = QLabel(title)
        title_label.setStyleSheet("""
            font-size: 14px;
            font-weight: bold;
            color: #2C3E50;
            padding: 2px;
        """)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setWordWrap(True)
        
        value_label = QLabel("Not Connected")
        value_label.setStyleSheet("""
            background-color: white;
            padding: 8px;
            border-radius: 8px;
            font-size: 14px;
            font-weight: bold;
            border: 1px solid #E9ECEF;
        """)
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setWordWrap(True)
        
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        
        if title == "pH Value":
            self.ph_value = value_label
        else:
            self.tds_value = value_label
        
        return widget

    def update_sensor_display(self, data):
        try:
            if 'error' in data:
                self.ph_value.setText("Not Connected")
                self.tds_value.setText("Not Connected")
                logging.warning("Sensor connection error")
            else:
                ph_value = data.get('ph', 'Error')
                tds_value = data.get('tds', 'Error')
                self.ph_value.setText(f"{ph_value}")
                self.tds_value.setText(f"{tds_value}")
                logging.debug(f"Updated sensor values - pH: {ph_value}, TDS: {tds_value}")
        except Exception as e:
            logging.error(f"Error updating sensor display: {e}")
        
    def closeEvent(self, event):
        try:
            if hasattr(self, 'sensor_thread'):
                self.sensor_thread.stop()
                self.sensor_thread.wait()
            super().closeEvent(event)
            logging.info("Monitoring widget closed properly")
        except Exception as e:
            logging.error(f"Error during monitoring widget closure: {e}")

class WaterSustainabilityApp(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            # Initialize water controller
            self.water_controller = WaterController()
            
            # Initialize UI
            self.initUI()
            logging.info("Water Sustainability App initialized successfully")
        except Exception as e:
            logging.critical(f"Failed to initialize application: {e}")
            raise

    def initUI(self):
        self.setWindowTitle("Innovative Aqua Solution")
        self.setStyleSheet("background-color: #E3F2FD;")
        
        # Get screen size and set window size
        screen = QApplication.desktop().screenGeometry()
        width = int(screen.width() * 0.8)
        height = int(screen.height() * 0.8)
        self.setMinimumSize(1366, 768)
        self.resize(width, height)
        
        # Main widget setup
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)
        
        # Content area
        content = QWidget()
        content.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout = QHBoxLayout(content)
        content_layout.setSpacing(20)
        
        # Create and setup left panel
        left_panel = self.create_left_panel()
        
        # Create and setup right panel
        right_panel = self.create_right_panel()
        
        # Add panels to content layout
        content_layout.addWidget(left_panel, 7)
        content_layout.addWidget(right_panel, 0)
        
        # Add content to main layout
        main_layout.addWidget(content)
        
        # Setup video stream
        self.setup_video()
        
        self.selected_size = None

    def create_left_panel(self):
        left_panel = QWidget()
        left_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(20)
        
        # Video container
        video_container = QWidget()
        video_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_container.setStyleSheet("""
            background-color: #F8F9FA;
            border-radius: 20px;
            border: 2px solid #E9ECEF;
            padding: 10px;
        """)
        video_layout = QVBoxLayout(video_container)
        
        self.video_label = QLabel()
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setMaximumSize(1280, 720)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("QLabel { background-color: #E9ECEF; border-radius: 10px; }")
        video_layout.addWidget(self.video_label)
        
        # Add video and monitoring widgets
        left_layout.addWidget(video_container, 7)
        left_layout.addWidget(MonitoringWidget(), 3)
        
        return left_panel

    def create_right_panel(self):
        right_panel = QWidget()
        right_panel.setFixedWidth(350)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(20)
        
        # Water selection section
        selection_widget = QWidget()
        selection_widget.setStyleSheet("""
            background-color: #F8F9FA;
            border-radius: 20px;
            border: 2px solid #E9ECEF;
            padding: 20px;
        """)
        selection_layout = QVBoxLayout(selection_widget)
        
        # Title
        selection_title = QLabel("Select Water Size")
        selection_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #2C3E50;
            font-family: 'Segoe UI', Arial;
        """)
        selection_title.setAlignment(Qt.AlignCenter)
        
        # Buttons grid
        buttons_grid = QGridLayout()
        buttons_grid.setSpacing(10)
        
        water_sizes = [
            ("100 ml", "1.png", 0, 0),
            ("350 ml", "2.png", 0, 1),
            ("600 ml", "3.png", 1, 0),
            ("1 Liter", "4.png", 1, 1)
        ]
        
        self.size_buttons = []
        for size, image_path, row, col in water_sizes:
            try:
                if os.path.exists(image_path):
                    btn = WaterButton(size, image_path)
                    btn.clicked.connect(lambda checked, s=size, b=btn: self.on_size_selected(s, b))
                    buttons_grid.addWidget(btn, row, col)
                    self.size_buttons.append(btn)
                else:
                    logging.warning(f"Image not found: {image_path}")
            except Exception as e:
                logging.error(f"Error creating water button: {e}")
        
        selection_layout.addWidget(selection_title)
        selection_layout.addLayout(buttons_grid)
        selection_layout.addStretch()
        
        # Machine control widget
        self.machine_widget = MachineWidget(self.water_controller)
        self.machine_widget.start_button.clicked.connect(self.start_filling)
        self.water_controller.update_progress.connect(self.machine_widget.update_progress)
        self.water_controller.filling_complete.connect(self.machine_widget.complete_filling)
        
        right_layout.addWidget(selection_widget)
        right_layout.addWidget(self.machine_widget)
        right_layout.addStretch()
        
        return right_panel

    def on_size_selected(self, size, clicked_button):
        try:
            for btn in self.size_buttons:
                if btn != clicked_button:
                    btn.setChecked(False)
            
            self.selected_size = size if clicked_button.isChecked() else None
            self.machine_widget.start_button.setEnabled(self.selected_size is not None)
            logging.info(f"Water size selected: {self.selected_size}")
        except Exception as e:
            logging.error(f"Error in size selection: {e}")

    def start_filling(self):
        if self.selected_size:
            try:
                logging.info(f"Starting filling process for {self.selected_size}")
                self.machine_widget.start_filling(self.selected_size)
            except Exception as e:
                logging.error(f"Error starting filling process: {e}")
                self.machine_widget.reset()

    def setup_video(self):
        try:
            video_path = "yqq.mp4"
            if not os.path.exists(video_path):
                self.video_label.setText("Video not found")
                logging.error(f"Video file not found: {video_path}")
                return
                
            self.video_thread = VideoThread(video_path)
            self.video_thread.frame_ready.connect(self.update_video_frame, Qt.QueuedConnection)
            self.video_thread.start()
            logging.info("Video stream setup completed")
            
        except Exception as e:
            logging.error(f"Error setting up video: {e}")
            self.video_label.setText(f"Error: {str(e)}")

    def update_video_frame(self, image):
        try:
            if not self.video_label or not image:
                return
                
            label_size = self.video_label.size()
            if not label_size.isValid():
                return
                
            scaled_pixmap = QPixmap.fromImage(image).scaled(
                label_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            if not scaled_pixmap.isNull():
                self.video_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            logging.error(f"Error updating video frame: {e}")

    def resizeEvent(self, event):
        try:
            super().resizeEvent(event)
            if hasattr(self, 'video_label'):
                new_width = min(1280, int(self.width() * 0.6))
                new_height = min(720, int(self.height() * 0.6))
                self.video_label.setMinimumSize(new_width, new_height)
        except Exception as e:
            logging.error(f"Error in resize event: {e}")

    def closeEvent(self, event):
        try:
            if hasattr(self, 'video_thread'):
                self.video_thread.stop()
                self.video_thread.wait()
            self.water_controller.cleanup()
            logging.info("Application closed properly")
            super().closeEvent(event)
        except Exception as e:
            logging.error(f"Error during application closure: {e}")

if __name__ == '__main__':
    try:
        # Initialize logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('vending_machine.log'),
                logging.StreamHandler()
            ]
        )
        
        # Create and start application
        app = QApplication(sys.argv)
        
        # Set application-wide font
        app.setFont(QFont('Segoe UI', 10))
        
        # Set style
        app.setStyle('Fusion')
        
        # Apply dark palette for better visibility on Raspberry Pi
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        app.setPalette(palette)
        
        # Set environment variables for Raspberry Pi
        os.environ['QT_QPA_PLATFORM'] = 'eglfs'  # Use EGLFS backend
        os.environ['QT_QPA_EGLFS_ALWAYS_SET_MODE'] = '1'  # Force mode setting
        
        # Initialize GPIO
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            logging.info("GPIO initialized successfully")
        except Exception as e:
            logging.error(f"Failed to initialize GPIO: {e}")
            sys.exit(1)
            
        # Create and show main window
        try:
            window = WaterSustainabilityApp()
            # Set window to fullscreen for Raspberry Pi
            window.showFullScreen()
            logging.info("Application window created and shown")
        except Exception as e:
            logging.critical(f"Failed to create main window: {e}")
            GPIO.cleanup()
            sys.exit(1)
            
        # Start event loop
        exit_code = app.exec_()
        
        # Cleanup
        try:
            GPIO.cleanup()
            logging.info("GPIO cleanup completed")
        except Exception as e:
            logging.error(f"Error during GPIO cleanup: {e}")
            
        sys.exit(exit_code)
        
    except KeyboardInterrupt:
        logging.info("Application terminated by user")
        try:
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            GPIO.cleanup()
            logging.info("Emergency motor stop and GPIO cleanup completed")
        except Exception as e:
            logging.error(f"Error during emergency cleanup: {e}")
        sys.exit(0)
        
    except Exception as e:
        logging.critical(f"Unhandled exception: {e}")
        try:
            GPIO.cleanup()
        except:
            pass
        sys.exit(1)