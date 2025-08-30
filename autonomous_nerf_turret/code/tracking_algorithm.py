import cv2
import numpy as np
from ultralytics import YOLO
from sort import Sort 
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import subprocess
import threading
import queue
import traceback
import sys
from collections import deque
import pigpio

# ----------------------- Raspberry Pi GPIO Setup -------------------
try:
    GPIO.setmode(GPIO.BCM)
    ENABLED = 0
    DISABLED = 1
    MOTOR_DELAY = 0.001 # Seconds

    # Base motor GPIOs
    BASE_DIR_PIN = 17
    BASE_STEP_PIN = 27
    BASE_EN_PIN = 22
    GPIO.setup(BASE_DIR_PIN, GPIO.OUT)
    GPIO.setup(BASE_STEP_PIN, GPIO.OUT)
    GPIO.setup(BASE_EN_PIN, GPIO.OUT)
    GPIO.output(BASE_EN_PIN, DISABLED)

    # Magazine motor GPIOs
    MAG_DIR_PIN = 5
    MAG_STEP_PIN = 6
    MAG_EN_PIN = 26
    GPIO.setup(MAG_DIR_PIN, GPIO.OUT)
    GPIO.setup(MAG_STEP_PIN, GPIO.OUT)
    GPIO.setup(MAG_EN_PIN, GPIO.OUT)
    GPIO.output(MAG_EN_PIN, DISABLED)

    # Button GPIO
    BUTTON_PIN = 23
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # ESC GPIOs
    LEFT_ESC_PIN = 16
    RIGHT_ESC_PIN = 20
    GPIO.setup(LEFT_ESC_PIN, GPIO.OUT)
    GPIO.setup(RIGHT_ESC_PIN, GPIO.OUT)

except Exception as e:
    print(f"GPIO Setup Error {e}")
    traceback.print_exc()
    sys.exit(1)

# ----------------------- pigpio Setup ------------------------------
try:
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(LEFT_ESC_PIN, 0) # Initialize pulses to 0 on boot
    pi.set_servo_pulsewidth(RIGHT_ESC_PIN, 0)
except Exception as e:
    print(f"ESC Setup Error {e}")
    sys.exit(1)

# ---------------------- Global Variables ---------------------------
motor_direction = 0                         # CW = 1, CCW = -1, STOP = 0
stop_threads = False                        # Flag to shut system down
shooting = False                            # Flag to start the BDSM
locked_target_id = None                     # Current locked target
locked_start_time = None                    # Time since the 2s shooting timer started
target_queue = deque()                      # Queue to hold targets
centre_tolerance = 15                       # Pixels from vertical centre considered "locked"
lock_duration = 2                           # Seconds needed to lock onto target before shooting
last_time = 0                               # Last time a person was detected
current_time = 0                            # Current system time
global_timeout = 5                          # Seconds needed for 0 detections before system motors disable
width, height, fps = 640, 480, 15           # Video frame parameters to prevent the pi from overloading
frame_centre_x = width // 2                 # The x-coordinate of the centre of the video frame used

# ----------------------- Motor Threads -----------------------------
def base_motor_function():
    global motor_direction, stop_threads
    while not stop_threads:
        try:
            if motor_direction != 0:
                GPIO.output(BASE_DIR_PIN, GPIO.HIGH if motor_direction > 0 else GPIO.LOW)
                GPIO.output(BASE_STEP_PIN, GPIO.HIGH)
                time.sleep(MOTOR_DELAY)
                GPIO.output(BASE_STEP_PIN, GPIO.LOW)
                time.sleep(MOTOR_DELAY)
                print(motor_direction) # test if it is rotating
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Base Motor Thread Error: {e}")
            traceback.print_exc()
            break

base_motor_thread = threading.Thread(target=base_motor_function)
base_motor_thread.daemon = True
base_motor_thread.start()

def mag_motor_function():
    global shooting, stop_threads
    while not stop_threads:
        try:
            if shooting:
                GPIO.output(MAG_DIR_PIN, 1)
                GPIO.output(MAG_STEP_PIN, GPIO.HIGH)
                time.sleep(MOTOR_DELAY)
                GPIO.output(MAG_STEP_PIN, GPIO.LOW)
                time.sleep(MOTOR_DELAY)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Magazine Motor Thread Error: {e}")
            traceback.print_exc()
            break

mag_motor_thread = threading.Thread(target=mag_motor_function)
mag_motor_thread.daemon = True
mag_motor_thread.start()

# ----------------------- ESC Shooting Thread -----------------------
def belt_shooting_function():
    global shooting, stop_threads
    last_update = 0
    update_interval = 0.05  # 50ms
    while not stop_threads:
        try:
            now = time.time()
            if shooting and now - last_update > update_interval:
                pi.set_servo_pulsewidth(LEFT_ESC_PIN, 2000)
                pi.set_servo_pulsewidth(RIGHT_ESC_PIN, 2000)
                last_update = now
            elif not shooting and now - last_update > update_interval:
                pi.set_servo_pulsewidth(LEFT_ESC_PIN, 0)
                pi.set_servo_pulsewidth(RIGHT_ESC_PIN, 0)
                last_update = now
            time.sleep(0.01)
        except Exception as e:
            print(f"ESC Thread Error: {e}")
            break
            
belt_shooting_thread = threading.Thread(target=belt_shooting_function)
belt_shooting_thread.daemon = True
belt_shooting_thread.start()

# ----------------------- OFF Button Thread -------------------------
def turn_system_off():
    global stop_threads
    while not stop_threads:
        try:
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                stop_threads = True
                time.sleep(0.02)
        except Exception as e:
            print(f"Off Button Thread Error: {e}")
            traceback.print_exc()
            break

# ----------------------- YOLO + SORT -------------------------------
try:
    model = YOLO("yolov8n.pt")
    tracker = Sort()
except Exception as e:
    print(f"YOLO or SORT Initialization Error: {e}")
    traceback.print_exc()
    sys.exit(1)

# ----------------------- Camera ------------------------------------
try:
    camera = Picamera2()
    camera.preview_configuration.main.size=(width,height)
    camera.preview_configuration.main.format="RGB888"
    camera.preview_configuration.align()
    camera.configure("preview")
    camera.start()
except Exception as e:
    print(f"Camera Initializaton Error: {e}")
    traceback.print_exc()
    sys.exit(1)

# ----------------------- FFmpeg RTSP -------------------------------
try:
    result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, check=True)
    ip_addr = result.stdout.strip()
except Exception as e:
    print(f"Error getting local IP address: {e}")
    traceback.print_exc()
    sys.exit(1)

ffmpeg_cmd = [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}',
        '-r', str(fps),
        '-i', '-',
        '-c:v', 'h264_v4l2m2m',
        '-preset', 'ultrafast',
        '-f', 'rtsp',
        '-rtsp_transport', 'tcp',
        f'rtsp://{ip_addr}:8554/nerfstream'
    ]
try:
    process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
except Exception as e:
    print(f"FFmpeg Process Error: {e}")
    traceback.print_exc()
    sys.exit(1)

# Shared frame queue for FFmpeg piping
frame_queue = queue.Queue(maxsize=2)

# ----------------------- Threading for RTSP stream ---------------------
def ffmpeg_streaming_thread(proc, q):
    while True:
        try:
            frame = q.get()
            if frame is None:  # Exit signal
                break
            proc.stdin.write(frame.tobytes())
        except Exception as e:
            print("FFmpeg error:", e)
            break

stream_thread = threading.Thread(target=ffmpeg_streaming_thread, args=(process, frame_queue))
stream_thread.daemon = True
stream_thread.start()

# ----------------------- Main Loop ---------------------------------
try:
    while not stop_threads:

        frame = camera.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        if not frame_queue.full():
            frame_queue.put_nowait(frame)

        results = model(frame, verbose=False)[0]

        # Detect humans
        detections = []
        for box in results.boxes.data.cpu().numpy():
            top_left_x, top_left_y, bottom_right_x, bottom_right_y, conf, cls = box
            if int(cls) == 0: # Only detect humans
                detections.append([top_left_x, top_left_y, bottom_right_x, bottom_right_y, conf])

        if len(detections) > 0: # Person detected
            last_time = time.time()
            GPIO.output(BASE_EN_PIN, ENABLED)
            GPIO.output(MAG_EN_PIN, ENABLED)

        detections = np.array(detections)
        tracks = tracker.update(detections)

        # Add new targets to queue in the order they appear
        current_ids = set()
        for track in tracks:
            _, _, _, _, track_id = track.astype(int)
            current_ids.add(track_id)
            if track_id not in target_queue and track_id != locked_target_id:
                target_queue.append(track_id)
        
        # Remove lost target if it's no longer detected
        if locked_target_id is not None and locked_target_id not in current_ids:
            locked_target_id = None
            lock_start_time = None
            if target_queue and target_queue[0] not in current_ids:
                target_queue.popleft()

        # If no locked target, pick from queue
        if locked_target_id is None and target_queue:
            locked_target_id = target_queue[0]
            lock_start_time = None
        
        motor_direction = 0  # Default stop

        for track in tracks:
            top_left_x, top_left_y, bottom_right_x, bottom_right_y, track_id = track.astype(int)
            x_centre = (top_left_x + bottom_right_x) // 2

            if track_id == locked_target_id:
                # Control motor based on position
                delta_x = x_centre - frame_centre_x
                if abs(delta_x) > centre_tolerance:
                    motor_direction = 1 if delta_x > 0 else -1
                    lock_start_time = None
                    shooting = False
                else:
                    motor_direction = 0
                    if lock_start_time is None:
                        lock_start_time = time.time()
                    elif time.time() - lock_start_time >= lock_duration:
                        shooting = True
                        # Remove curent target from queue after shooting
                        if target_queue and target_queue[0] == locked_target_id:
                            target_queue.popleft()
                        locked_target_id = None
                        lock_start_time = None

        current_time = time.time()

        # If no one detected after global timeout, disable motors
        if current_time - last_time > global_timeout:
            GPIO.output(BASE_EN_PIN, DISABLED)
            GPIO.output(MAG_EN_PIN, DISABLED)

except KeyboardInterrupt:
    print("Received keyboard interrupt, exiting.")

finally:
    stop_threads = True
    base_motor_thread.join()
    mag_motor_thread.join()
    belt_shooting_thread.join()
    frame_queue.put(None) # Signal FFmpeg thread to exit
    stream_thread.join()

    try:
        process.stdin.close()
    except Exception as e:
        print(f"Error Closing FFmpeg Frame Writing: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        process.wait()
    except Exception as e:
        print(f"Error waiting for FFmpeg Process: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"Error Closing GPIOs: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        pi.set_servo_pulsewidth(LEFT_ESC_PIN, 0)
        pi.set_servo_pulsewidth(RIGHT_ESC_PIN, 0)
        pi.stop()
    except Exception as e:
        print(f"pigpio Stop Error: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        camera.stop()
        camera.close()
    except Exception as e:
        print(f"Error Closing Camera: {e}")
        traceback.print_exc()
        sys.exit(1)

cv2.destroyAllWindows()
