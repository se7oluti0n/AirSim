
# AirSim PTZ Camera Simulator (Direct GStreamer & pynput)
#
# This script simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
# It launches a GStreamer pipeline as a separate process and pipes raw video
# frames to it for H.264 encoding and UDP streaming.
#
# This version uses pynput for keyboard control and does not require a GUI window.
#
# Author: Gemini
# Date: August 3, 2025

# --- Installation ---
# pip install airsim opencv-python numpy pynput

# --- Prerequisites ---
# - GStreamer must be installed on your system and accessible from the command line.

# --- How to Run ---
# 1. Start your AirSim environment with PX4 SITL.
# 2. Run this Python script.
# 3. Create a 'stream.sdp' file and open it with VLC to view the stream.
# 4. Make sure the terminal window running this script is in focus to use the PTZ controls.

import airsim
import cv2
import numpy as np
import time
import math
import subprocess
from pynput import keyboard

# --- Configuration ---
# Streaming Configuration
UDP_IP = "127.0.0.1"      # IP address to stream to (localhost)
UDP_PORT = 5000           # Port to stream to
IMG_WIDTH = 1280          # Video frame width
IMG_HEIGHT = 720          # Video frame height
FPS = 30                  # Frames per second

# PTZ Control Configuration
VEHICLE_NAME = "PX4" 
CAMERA_NAME = "0"       
PAN_SPEED = 2.0         
TILT_SPEED = 2.0        
ZOOM_SPEED = 2.0        
MIN_FOV = 15            
MAX_FOV = 90            

# --- Global State ---
# A set to hold the currently pressed keys
pressed_keys = set()
running = True

def on_press(key):
    """Callback function for when a key is pressed."""
    if hasattr(key, 'char'):
        pressed_keys.add(key.char)
    else:
        pressed_keys.add(key)

def on_release(key):
    """Callback function for when a key is released."""
    global running
    if hasattr(key, 'char'):
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    else:
        if key in pressed_keys:
            pressed_keys.remove(key)
    
    if key == keyboard.Key.esc:
        print("\nESC pressed. Shutting down...")
        running = False
        return False # Stop the listener

def main():
    """
    Main function to initialize AirSim, launch GStreamer, and handle all API calls.
    """
    global running
    # --- GStreamer Pipeline Command ---
    # This command reads raw video frames from stdin (fd=0), interprets them
    # as BGR video of a specific size and framerate, converts the color space,
    # encodes to H.264, and streams over UDP.
    gst_command = (
        f'gst-launch-1.0 -v fdsrc ! '
        # FIX: Corrected format to BGR (all caps)
        f'videoparse format=bgr width={IMG_WIDTH} height={IMG_HEIGHT} framerate={FPS}/1 ! '
        f'videoconvert ! '
        # FIX: Increased bitrate for better quality, added sync=false to udpsink for lower latency
        f'x264enc tune=zerolatency bitrate=4000 speed-preset=superfast ! '
        f'rtph264pay ! '
        f'udpsink host={UDP_IP} port={UDP_PORT} sync=false'
    )

    # --- Launch GStreamer as a Subprocess ---
    print(f"🚀 Launching GStreamer pipeline...")
    print(f"Streaming to udp://{UDP_IP}:{UDP_PORT}")
    try:
        gst_process = subprocess.Popen(gst_command, stdin=subprocess.PIPE, shell=True)
    except FileNotFoundError:
        print("Error: 'gst-launch-1.0' not found.")
        print("Please make sure GStreamer is installed and in your system's PATH.")
        return

    # --- Initialize AirSim Client ---
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        print("✅ Successfully connected to AirSim.")
    except Exception as e:
        print(f"Error: Could not connect to AirSim. Please ensure it is running.")
        print(f"Details: {e}")
        gst_process.stdin.close()
        gst_process.wait()
        return

    # --- Start Keyboard Listener ---
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\n--- PTZ Camera Controls ---")
    print("  Pan Left:    'a' or Left Arrow")
    print("  Pan Right:   'd' or Right Arrow")
    print("  Tilt Up:     'w' or Up Arrow")
    print("  Tilt Down:   's' or Down Arrow")
    print("  Zoom In:     'e' or '='")
    print("  Zoom Out:    'q' or '-'")
    print("  Exit:        'esc'")
    print("---------------------------\n")
    print("ℹ️  Focus this terminal window to use controls.")

    request = airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False)

    # --- PTZ State Variables ---
    pan = 0.0
    tilt = 0.0
    fov = MAX_FOV

    # --- Main Loop ---
    while running:
        start_time = time.time()

        # Get image from AirSim
        responses = client.simGetImages([request])
        response = responses[0]

        if response.image_data_uint8:
            img_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img_1d.reshape(response.height, response.width, 3)
            
            if response.width != IMG_WIDTH or response.height != IMG_HEIGHT:
                img_bgr = cv2.resize(img_bgr, (IMG_WIDTH, IMG_HEIGHT))
            
            # Write the raw frame data to the GStreamer process
            try:
                gst_process.stdin.write(img_bgr.tobytes())
            except (BrokenPipeError, IOError):
                print("GStreamer process has closed. Shutting down.")
                running = False
                break
        
        # Update PTZ state based on keyboard input
        if 'a' in pressed_keys or keyboard.Key.left in pressed_keys: pan -= PAN_SPEED
        if 'd' in pressed_keys or keyboard.Key.right in pressed_keys: pan += PAN_SPEED
        if 'w' in pressed_keys or keyboard.Key.up in pressed_keys: tilt -= TILT_SPEED
        if 's' in pressed_keys or keyboard.Key.down in pressed_keys: tilt += TILT_SPEED
        if 'e' in pressed_keys or '=' in pressed_keys: fov -= ZOOM_SPEED
        if 'q' in pressed_keys or '-' in pressed_keys: fov += ZOOM_SPEED

        # Clamp values
        tilt = max(-89.0, min(89.0, tilt))
        fov = max(MIN_FOV, min(MAX_FOV, fov))
        pan %= 360
        
        # Apply PTZ updates
        try:
            pose = airsim.Pose()
            pose.position.x_val = 0.0
            pose.position.y_val = 0.0
            pose.position.z_val = 0.0
            pose.orientation = airsim.to_quaternion(math.radians(tilt), 0, math.radians(pan))
            client.simSetCameraPose(CAMERA_NAME, pose, vehicle_name=VEHICLE_NAME)
            client.simSetCameraFov(CAMERA_NAME, fov, vehicle_name=VEHICLE_NAME)
        except Exception as e:
            if running:
                print(f"Error updating camera: {e}")

        # Regulate loop speed
        elapsed_time = time.time() - start_time
        sleep_time = (1.0 / FPS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- Cleanup ---
    print("Cleaning up resources...")
    listener.stop()
    gst_process.stdin.close()
    gst_process.wait()
    print("✅ Application has been shut down gracefully.")

if __name__ == "__main__":
    main()
