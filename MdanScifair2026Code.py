#!/usr/bin/env python3
"""
Raspberry Pi Zero W Continuous Video Capture with Switch Control and MPU6050 Data Logging
Records video while switch is ON, stops when switch is OFF
Logs accelerometer data to text file during recording
Flipping switch ON again starts a new recording
Compatible with Debian Bookworm and rpicam with Camera v3
"""

import subprocess
import time
import os
from datetime import datetime
import RPi.GPIO as GPIO
import signal
import sys
import threading
import board
import adafruit_mpu6050

# GPIO Configuration
SWITCH_PIN = 17  # Change this to your GPIO pin number for recording switch
SHUTDOWN_PIN = 23  # Change this to your GPIO pin number for shutdown button
USE_PULL_UP = True  # Set to True if switch connects to ground, False if connects to 3.3V

# Global variables
recording_process = None
data_logging_active = False
data_log_thread = None
mpu = None
shutdown_button_last_state = GPIO.HIGH  # Track button state

def shutdown_button_pressed():
    """Handle shutdown button press"""
    global recording_process, data_logging_active, data_log_thread
    
    print("\n" + "="*50)
    print("SHUTDOWN BUTTON PRESSED!")
    print("Stopping all recordings and shutting down safely...")
    print("="*50)
    
    # Stop data logging
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=3)
        print("Data logging stopped safely.")
    
    # Stop video recording
    if recording_process:
        print("Stopping video recording...")
        try:
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait(timeout=5)
        except:
            pass
        print("Video recording stopped safely.")
    
    # Clean up GPIO
    GPIO.cleanup()
    print("GPIO cleaned up.")
    
    # Give a moment for all processes to finish
    time.sleep(1)
    
    print("Initiating system shutdown...")
    # Use subprocess to call shutdown command
    subprocess.call(['sudo', 'shutdown', '-h', 'now'])
    
    # Exit the program
    sys.exit(0)

def check_shutdown_button():
    """Check if shutdown button is pressed (polling method)"""
    global shutdown_button_last_state
    
    current_state = GPIO.input(SHUTDOWN_PIN)
    
    # Button pressed when it goes from HIGH to LOW
    if shutdown_button_last_state == GPIO.HIGH and current_state == GPIO.LOW:
        shutdown_button_last_state = current_state
        shutdown_button_pressed()
    
    shutdown_button_last_state = current_state

def setup_gpio():
    """Setup GPIO for switch and shutdown button"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # Disable warnings
    
    # Setup recording switch
    if USE_PULL_UP:
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    else:
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Setup shutdown button (always with pull-up) - using polling instead of edge detection
    GPIO.setup(SHUTDOWN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def setup_mpu6050():
    """Setup MPU6050 accelerometer"""
    global mpu
    try:
        i2c = board.I2C()
        mpu = adafruit_mpu6050.MPU6050(i2c)
        print("MPU6050 initialized successfully")
        return True
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        print("Continuing without accelerometer data...")
        return False

def is_switch_on():
    """Check if switch is in ON position"""
    if USE_PULL_UP:
        # Switch connects to ground, so LOW = ON
        return GPIO.input(SWITCH_PIN) == GPIO.LOW
    else:
        # Switch connects to 3.3V, so HIGH = ON
        return GPIO.input(SWITCH_PIN) == GPIO.HIGH

def check_camera():
    """Verify camera is detected"""
    try:
        result = subprocess.run(
            ["rpicam-hello", "--list-cameras"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if "Available cameras" in result.stdout:
            print("Camera detected successfully")
            return True
        else:
            print("No camera detected!")
            return False
    except Exception as e:
        print(f"Error checking camera: {e}")
        return False

def log_accelerometer_data(txt_filepath):
    """Continuously log accelerometer data to text file"""
    global data_logging_active, mpu
    
    if mpu is None:
        print("MPU6050 not available, skipping data logging")
        return
    
    try:
        with open(txt_filepath, 'w') as f:
            # Write header
            f.write("Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Temperature\n")
            f.flush()
            
            print(f"Logging accelerometer data to: {txt_filepath}")
            
            while data_logging_active:
                try:
                    # Get current time
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    # Read sensor data
                    accel_x, accel_y, accel_z = mpu.acceleration
                    gyro_x, gyro_y, gyro_z = mpu.gyro
                    temp = mpu.temperature
                    
                    # Write to file
                    f.write(f"{timestamp},{accel_x:.3f},{accel_y:.3f},{accel_z:.3f},"
                           f"{gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f},{temp:.2f}\n")
                    f.flush()
                    
                    # Wait 0.5 seconds before next reading
                    time.sleep(0.5)
                    
                except Exception as e:
                    print(f"Error reading sensor: {e}")
                    time.sleep(0.5)
                    
    except Exception as e:
        print(f"Error writing to log file: {e}")

def start_recording():
    """Start recording video and logging accelerometer data"""
    global recording_process, data_logging_active, data_log_thread
    
    # Create Videos directory if it doesn't exist
    videos_dir = os.path.expanduser("~/Videos")
    os.makedirs(videos_dir, exist_ok=True)
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    h264_filename = f"video_{timestamp}.h264"
    txt_filename = f"video_{timestamp}.txt"
    h264_filepath = os.path.join(videos_dir, h264_filename)
    txt_filepath = os.path.join(videos_dir, txt_filename)
    
    print(f"Starting recording...")
    print(f"Video: {h264_filepath}")
    print(f"Data log: {txt_filepath}")
    
    # Start video recording
    recording_process = subprocess.Popen([
        "rpicam-vid",
        "-t", "0",  # Infinite recording
        "-o", h264_filepath,
        "--nopreview",
        "--width", "720",
        "--height", "720",
        "--framerate", "30"
    ])
    
    # Start accelerometer data logging in separate thread
    data_logging_active = True
    data_log_thread = threading.Thread(target=log_accelerometer_data, args=(txt_filepath,))
    data_log_thread.start()
    
    return h264_filepath, txt_filepath

def stop_recording(h264_filepath, txt_filepath):
    """Stop recording and convert to MP4, close data log"""
    global recording_process, data_logging_active, data_log_thread
    
    if recording_process is None:
        return
    
    print("Stopping recording and data logging...")
    
    # Stop data logging thread
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=2)
    
    print("Data logging stopped, file closed.")
    
    # Send SIGINT to gracefully stop rpicam-vid
    recording_process.send_signal(signal.SIGINT)
    recording_process.wait()
    recording_process = None
    
    print("Video recording stopped!")
    
    # Wait a moment for file to be fully written
    time.sleep(1)
    
    # Check if h264 file exists and has content
    if not os.path.exists(h264_filepath) or os.path.getsize(h264_filepath) == 0:
        print("Error: Recording file is empty or doesn't exist")
        return
    
    # Convert to MP4
    mp4_filepath = h264_filepath.replace(".h264", ".mp4")
    print(f"Converting to MP4 format...")
    
    result = subprocess.run([
        "/usr/bin/ffmpeg",
        "-framerate", "30",
        "-i", h264_filepath,
        "-c", "copy",
        mp4_filepath,
        "-y"
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"FFmpeg conversion failed!")
        print(f"Error: {result.stderr}")
        print(f"H.264 file saved at: {h264_filepath}")
    else:
        # Remove the h264 file after successful conversion
        os.remove(h264_filepath)
        print(f"Conversion complete! Original H.264 file removed.")
        print(f"Final video saved to: {mp4_filepath}")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global recording_process, data_logging_active, data_log_thread
    print("\nShutting down...")
    
    # Stop data logging
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=2)
    
    # Stop video recording
    if recording_process:
        recording_process.terminate()
        recording_process.wait()
    
    GPIO.cleanup()
    sys.exit(0)

def main():
    global recording_process
    
    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check camera
    if not check_camera():
        print("Cannot proceed - camera not detected")
        return
    
    # Setup MPU6050
    setup_mpu6050()
    
    setup_gpio()
    
    print("Ready!")
    print(f"- Recording switch on GPIO {SWITCH_PIN}: Flip ON to start recording, OFF to stop.")
    print(f"- Shutdown button on GPIO {SHUTDOWN_PIN}: Press to safely shutdown Pi.")
    print("Press Ctrl+C to exit program without shutdown.")
    print("")
    
    current_video_filepath = None
    current_data_filepath = None
    
    while True:
        # Check shutdown button first
        check_shutdown_button()
        
        if is_switch_on():
            # Switch is ON
            if recording_process is None:
                # Not currently recording, start new recording
                print("\n=== Switch ON ===")
                time.sleep(1)  # Short delay to debounce
                current_video_filepath, current_data_filepath = start_recording()
        else:
            # Switch is OFF
            if recording_process is not None:
                # Currently recording, stop it
                print("\n=== Switch OFF ===")
                stop_recording(current_video_filepath, current_data_filepath)
                current_video_filepath = None
                current_data_filepath = None
        
        time.sleep(0.1)  # Check switch state 10 times per second

if __name__ == "__main__":
    try:
        main()
    finally:
        data_logging_active = False
        if data_log_thread is not None:
            data_log_thread.join(timeout=2)
        if recording_process:
            recording_process.terminate()
            recording_process.wait()
        GPIO.cleanup()