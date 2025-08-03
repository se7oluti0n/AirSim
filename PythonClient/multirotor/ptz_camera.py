
# AirSim PTZ Camera Simulator
#
# This script simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
# It performs two main tasks concurrently:
# 1. Streams video from an AirSim camera to a UDP port, encoded in H.264.
# 2. Listens for keyboard input to control the camera's pan, tilt, and zoom.
#
# Author: Gemini
# Date: August 2, 2025

# --- Installation ---
# You need to install the following Python packages:
# pip install airsim opencv-python numpy keyboard

# --- AirSim Setup ---
# 1. Make sure you have an AirSim environment (e.g., Blocks, Neighborhood) running.
# 2. Your AirSim settings.json file should be configured to enable the API.
#    A minimal settings.json looks like this:
#    {
#      "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
#      "SettingsVersion": 1.2,
#      "SimMode": "Multirotor",
#      "Vehicles": {
#        "Drone1": {
#          "VehicleType": "SimpleFlight",
#          "X": 0, "Y": 0, "Z": 0
#        }
#      }
#    }

# --- How to Run ---
# 1. Start your AirSim environment.
# 2. Run this Python script from your terminal: python your_script_name.py
# 3. Open a network stream in a video player like VLC with the following URL:
#    udp://@:5000
# 4. Focus on the terminal window running the script and use the control keys.

import airsim
import cv2
import numpy as np
import threading
import time
import keyboard
import math

# --- Configuration ---
# Streaming Configuration
UDP_IP = "127.0.0.1"  # IP address to stream to (localhost)
UDP_PORT = 5000         # Port to stream to
IMG_WIDTH = 1280        # Video frame width
IMG_HEIGHT = 720        # Video frame height
FPS = 30                # Frames per second

# PTZ Control Configuration
VEHICLE_NAME = "Drone1" # Must match the name in your AirSim settings.json
CAMERA_NAME = "0"       # "0" is the front-center camera
PAN_SPEED = 2.0         # Degrees per step for pan
TILT_SPEED = 2.0        # Degrees per step for tilt
ZOOM_SPEED = 2.0        # Degrees per step for zoom (FoV change)
MIN_FOV = 15            # Minimum Field of View (max zoom)
MAX_FOV = 90            # Maximum Field of View (min zoom)

# --- Global State ---
# This dictionary will hold the current PTZ state, shared between threads.
ptz_state = {
    'pan': 0.0,   # Current pan in degrees
    'tilt': 0.0,  # Current tilt in degrees
    'fov': MAX_FOV, # Current Field of View
    'running': True # Flag to stop threads gracefully
}

def setup_video_stream():
    """
    Sets up the GStreamer pipeline for H.264 encoding and UDP streaming.
    This function creates an OpenCV VideoWriter object.
    """
    # GStreamer pipeline for encoding and streaming
    # appsrc:      Gets raw frames from this script
    # videoconvert: Converts color space
    # x264enc:     Encodes the stream to H.264. 'tune=zerolatency' is crucial for live streaming.
    # rtph264pay:  Creates RTP packets for H.264
    # udpsink:     Sends the packets over UDP
    gst_pipeline = (
        f"appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! "
        f"rtph264pay ! udpsink host={UDP_IP} port={UDP_PORT}"
    )
    
    video_writer = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, FPS, (IMG_WIDTH, IMG_HEIGHT), True)
    
    if not video_writer.isOpened():
        print("Error: Failed to open GStreamer pipeline. Make sure GStreamer is installed correctly.")
        return None
        
    print(f"✅ GStreamer pipeline initialized. Streaming to udp://{UDP_IP}:{UDP_PORT}")
    return video_writer

def video_streamer_thread(client, video_writer):
    """
    This function runs in a separate thread. It continuously captures images
    from AirSim, processes them, and writes them to the video stream.
    """
    print("🚀 Video streaming thread started.")
    
    # Define the request for a scene image
    request = airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False)

    while ptz_state['running']:
        start_time = time.time()
        
        # Get image from AirSim
        responses = client.simGetImages([request])
        response = responses[0]

        if response.image_data_uint8:
            # Convert the raw image data to a NumPy array
            img_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            # Reshape the array to an image (3 channels for BGR)
            img_bgr = img_1d.reshape(response.height, response.width, 3)
            
            # Resize if necessary to match our streaming resolution
            if response.width != IMG_WIDTH or response.height != IMG_HEIGHT:
                img_bgr = cv2.resize(img_bgr, (IMG_WIDTH, IMG_HEIGHT))

            # Write the frame to the GStreamer pipeline
            video_writer.write(img_bgr)

        # Regulate the loop to approximate the desired FPS
        elapsed_time = time.time() - start_time
        sleep_time = (1.0 / FPS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    print("🛑 Video streaming thread stopped.")
    video_writer.release()


def ptz_controller():
    """
    This function runs in the main thread. It listens for keyboard inputs
    and updates the shared ptz_state dictionary.
    """
    print("\n--- PTZ Camera Controls ---")
    print("  Pan Left:    'a' or Left Arrow")
    print("  Pan Right:   'd' or Right Arrow")
    print("  Tilt Up:     'w' or Up Arrow")
    print("  Tilt Down:   's' or Down Arrow")
    print("  Zoom In:     '+' or '='")
    print("  Zoom Out:    '-'")
    print("  Exit:        'esc'")
    print("---------------------------\n")
    print("ℹ️  Focus this terminal window to use controls.")

    while ptz_state['running']:
        # Pan controls
        if keyboard.is_pressed('a') or keyboard.is_pressed('left'):
            ptz_state['pan'] -= PAN_SPEED
        if keyboard.is_pressed('d') or keyboard.is_pressed('right'):
            ptz_state['pan'] += PAN_SPEED
            
        # Tilt controls
        if keyboard.is_pressed('w') or keyboard.is_pressed('up'):
            ptz_state['tilt'] -= TILT_SPEED
        if keyboard.is_pressed('s') or keyboard.is_pressed('down'):
            ptz_state['tilt'] += TILT_SPEED
            
        # Zoom controls
        if keyboard.is_pressed('+') or keyboard.is_pressed('='):
            ptz_state['fov'] -= ZOOM_SPEED
        if keyboard.is_pressed('-'):
            ptz_state['fov'] += ZOOM_SPEED
            
        # Exit control
        if keyboard.is_pressed('esc'):
            ptz_state['running'] = False
            print("\nESC pressed. Shutting down...")
            
        # Clamp values to their limits
        ptz_state['tilt'] = max(-89.0, min(89.0, ptz_state['tilt'])) # Limit tilt to avoid gimbal lock issues
        ptz_state['fov'] = max(MIN_FOV, min(MAX_FOV, ptz_state['fov']))
        ptz_state['pan'] %= 360 # Keep pan within 0-360 range

        time.sleep(0.05) # Small delay to prevent high CPU usage

def main():
    """
    Main function to initialize AirSim client, start threads,
    and apply PTZ updates.
    """
    # Initialize AirSim client
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        client.enableApiControl(True, VEHICLE_NAME)
        print("✅ Successfully connected to AirSim and enabled API control.")
    except Exception as e:
        print(f"Error: Could not connect to AirSim. Please ensure it is running.")
        print(f"Details: {e}")
        return

    # Setup video streaming
    video_writer = setup_video_stream()
    if not video_writer:
        client.enableApiControl(False, VEHICLE_NAME)
        return

    # Start the video streaming thread
    stream_thread = threading.Thread(target=video_streamer_thread, args=(client, video_writer))
    stream_thread.start()
    
    # Start the keyboard listener in the main thread
    # This function will block until ESC is pressed
    ptz_controller_thread = threading.Thread(target=ptz_controller)
    ptz_controller_thread.start()

    # Main loop to apply PTZ updates from the shared state
    last_pan, last_tilt, last_fov = -1, -1, -1
    
    while ptz_state['running']:
        pan, tilt, fov = ptz_state['pan'], ptz_state['tilt'], ptz_state['fov']
        
        # Only update AirSim if there's a change, to reduce API calls
        if pan != last_pan or tilt != last_tilt:
            # AirSim uses quaternions for orientation. We convert from Euler angles (pitch, roll, yaw).
            # Pan corresponds to Yaw, Tilt corresponds to Pitch. Roll is kept at 0.
            q = airsim.to_quaternion(math.radians(tilt), 0, math.radians(pan))
            client.simSetCameraOrientation(CAMERA_NAME, q, vehicle_name=VEHICLE_NAME)
            last_pan, last_tilt = pan, tilt
            
        if fov != last_fov:
            client.simSetCameraFov(CAMERA_NAME, fov, vehicle_name=VEHICLE_NAME)
            last_fov = fov
            
        time.sleep(0.02) # Update rate for applying camera changes

    # Cleanup
    print("Waiting for threads to join...")
    stream_thread.join()
    ptz_controller_thread.join()
    client.enableApiControl(False, VEHICLE_NAME)
    print("✅ Application has been shut down gracefully.")


if __name__ == "__main__":
    print(cv2.getBuildInformation())
    main()
