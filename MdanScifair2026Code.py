#!/usr/bin/env python3

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

# config
SWITCH_PIN = 17  
SHUTDOWN_PIN = 23  
LED_PIN = 22 
USE_PULL_UP = True  
recording_process = None
data_logging_active = False
data_log_thread = None
mpu = None
shutdown_button_last_state = GPIO.HIGH  

# accel offsets
ACCEL_OFFSET_X = 0.194
ACCEL_OFFSET_Y = 0.319  
ACCEL_OFFSET_Z = -1.432  




def shutdown_button_pressed():
    # handle button press
    global recording_process, data_logging_active, data_log_thread
    
    print("\n" + "="*50)
    print("stopping")
    print("="*50)
    
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=3)
        print("data logging stopped")
    
    if recording_process:
        print("stopped video recording")
        try:
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait(timeout=5)
        except:
            pass
    
    GPIO.cleanup()
    print("GPIO cleaned up.")
    
    time.sleep(1)
    
    print("shutting down")
    subprocess.call(['sudo', 'shutdown', '-h', 'now'])
    
    sys.exit(0)

def check_shutdown_button():
    """Check if shutdown button is pressed (polling method)"""
    global shutdown_button_last_state
    
    current_state = GPIO.input(SHUTDOWN_PIN)
    
    if shutdown_button_last_state == GPIO.HIGH and current_state == GPIO.LOW:
        shutdown_button_last_state = current_state
        shutdown_button_pressed()
    
    shutdown_button_last_state = current_state

def setup_gpio():
    """Setup GPIO for switch and shutdown button"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # Disable warnings
    
    # setup recording switch
    if USE_PULL_UP:
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    else:
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # setup shutdown button
    GPIO.setup(SHUTDOWN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # setup LED 
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW) 

def setup_mpu6050():
    """Setup MPU6050 accelerometer"""
    global mpu
    try:
        i2c = board.I2C()
        mpu = adafruit_mpu6050.MPU6050(i2c)
        return True
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        return False

def is_switch_on():
    """Check if switch is in ON position"""
    if USE_PULL_UP:
        return GPIO.input(SWITCH_PIN) == GPIO.LOW
    else:
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
            print("camera works")
            return True
        else:
            print("camera doesnt work")
            return False
    except Exception as e:
        print(f"Error checking camera: {e}")
        return False

def log_accelerometer_data(txt_filepath):
    """Continuously log accelerometer data to text file"""
    global data_logging_active, mpu
    
    if mpu is None:
        print("MPU6050 failed")
        return
    
    try:
        with open(txt_filepath, 'w') as f:
            f.write("Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Temperature\n")
            f.flush()
            
            print(f"logging data to: {txt_filepath}")
            print(f"offsets - X: {ACCEL_OFFSET_X}, Y: {ACCEL_OFFSET_Y}, Z: {ACCEL_OFFSET_Z}")
            
            while data_logging_active:
                try:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    accel_x_raw, accel_y_raw, accel_z_raw = mpu.acceleration
                    gyro_x, gyro_y, gyro_z = mpu.gyro
                    temp = mpu.temperature
                    
                    accel_x = accel_x_raw - ACCEL_OFFSET_X
                    accel_y = accel_y_raw - ACCEL_OFFSET_Y
                    accel_z = accel_z_raw - ACCEL_OFFSET_Z
                
                    f.write(f"{timestamp},{accel_x:.3f},{accel_y:.3f},{accel_z:.3f},"
                           f"{gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f},{temp:.2f}\n")
                    f.flush()
                    
                    time.sleep(0.5)
                    
                except Exception as e:
                    print(f"Error reading sensor: {e}")
                    time.sleep(0.5)
                    
    except Exception as e:
        print(f"Error writing to log file: {e}")

def start_recording():
    """Start recording video and logging accelerometer data"""
    global recording_process, data_logging_active, data_log_thread
    
    videos_dir = os.path.expanduser("~/Videos")
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    h264_filename = f"video_{timestamp}.h264"
    txt_filename = f"video_{timestamp}.txt"
    h264_filepath = os.path.join(videos_dir, h264_filename)
    txt_filepath = os.path.join(videos_dir, txt_filename)
    
    print(f"starting recording")
    print(f"video: {h264_filepath}")
    print(f"data log: {txt_filepath}")
    
    GPIO.output(LED_PIN, GPIO.HIGH)
    
    # start video recording 
    recording_process = subprocess.Popen([
        "rpicam-vid",
        "-t", "0",  
        "-o", h264_filepath,
        "--nopreview",
        "--width", "720",
        "--height", "720",
        "--framerate", "30"
    ])
    
    # start accelerometer logging 
    data_logging_active = True
    data_log_thread = threading.Thread(target=log_accelerometer_data, args=(txt_filepath,))
    data_log_thread.start()
    
    return h264_filepath, txt_filepath

def stop_recording(h264_filepath, txt_filepath):
    """Stop recording and convert to MP4, close data log"""
    global recording_process, data_logging_active, data_log_thread
    
    if recording_process is None:
        return
    
    print("stopping recording and logging")
    
    # turn off LED
    GPIO.output(LED_PIN, GPIO.LOW)
    
    # stop logging 
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=2)
    
    print("logging stopped, file closed")
    
    # stop rpicam
    recording_process.send_signal(signal.SIGINT)
    recording_process.wait()
    recording_process = None
    
    print("recording stopped")
    
    time.sleep(1)
    
    # check if file exists
    if not os.path.exists(h264_filepath) or os.path.getsize(h264_filepath) == 0:
        print("Error: file doesnt work")
        return
    
    # convert
    mp4_filepath = h264_filepath.replace(".h264", ".mp4")
    print(f"converting file")
    
    result = subprocess.run([
        "/usr/bin/ffmpeg",
        "-framerate", "30",
        "-i", h264_filepath,
        "-c", "copy",
        mp4_filepath,
        "-y"
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"conversion failed")
        print(f"Error: {result.stderr}")
        print(f"H.264 file saved at: {h264_filepath}")
    else:
        # remove h264
        os.remove(h264_filepath)
        print(f"conversion complete H.264 file removed.")
        print(f"video saved to: {mp4_filepath}")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global recording_process, data_logging_active, data_log_thread
    print("\nShutting down")
    
    # stop logging
    data_logging_active = False
    if data_log_thread is not None:
        data_log_thread.join(timeout=2)
    
    # stop recording
    if recording_process:
        recording_process.terminate()
        recording_process.wait()
    
    GPIO.cleanup()
    sys.exit(0)

def main():
    global recording_process
    
    # cntrl c support
    signal.signal(signal.SIGINT, signal_handler)
    
    # check camera
    if not check_camera():
        print("cannot proceed - no cameara")
        return
    
    # setup accel
    setup_mpu6050()
    
    setup_gpio()
    
    current_video_filepath = None
    current_data_filepath = None
    
    while True:
        # check button
        check_shutdown_button()
        
        if is_switch_on():
            if recording_process is None:
                print("\nSwitch ON")
                time.sleep(1)  # Short delay to debounce
                current_video_filepath, current_data_filepath = start_recording()
        else:
            if recording_process is not None:
                print("\nSwitch OFF")
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
        GPIO.output(LED_PIN, GPIO.LOW)  # Make sure LED is off
        GPIO.cleanup()
