
# AirSim PTZ Camera Simulator (UDP/JPEG Streaming Method)
#
# This script simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
# It streams a sequence of JPEG-compressed images over a raw UDP socket.
# This method does not require FFmpeg or GStreamer support in OpenCV.
#
# It performs two main tasks concurrently:
# 1. Streams JPEG images from an AirSim camera to a UDP port.
# 2. Listens for keyboard input to control the camera's pan, tilt, and zoom.
#
# Author: Gemini
# Date: August 2, 2025

# --- How to Run ---
# 1. Start your AirSim environment.
# 2. On Linux, run this script with sudo: sudo python3 your_script_name.py
# 3. In a SEPARATE terminal, run the 'udp_jpeg_client.py' script to view the stream.
# 4. Focus on the terminal window running THIS script to use the PTZ controls.

import airsim
import cv2
import numpy as np
import threading
import time
import keyboard
import math
import socket

# --- Configuration ---
# Streaming Configuration
UDP_IP = "127.0.0.1"    # IP address to stream to (localhost)
UDP_PORT = 5000           # Port to stream to
IMG_WIDTH = 1024          # Video frame width
IMG_HEIGHT = 576          # Video frame height
JPEG_QUALITY = 80         # JPEG quality (0-100), lower is smaller size
FPS = 30                  # Frames per second

# PTZ Control Configuration
VEHICLE_NAME = "Drone1" # Must match the name in your AirSim settings.json
CAMERA_NAME = "0"       # "0" is the front-center camera
PAN_SPEED = 2.0         # Degrees per step for pan
TILT_SPEED = 2.0        # Degrees per step for tilt
ZOOM_SPEED = 2.0        # Degrees per step for zoom (FoV change)
MIN_FOV = 15            # Minimum Field of View (max zoom)
MAX_FOV = 90            # Maximum Field of View (min zoom)

# --- Global State ---
ptz_state = {
    'pan': 0.0,
    'tilt': 0.0,
    'fov': MAX_FOV,
    'running': True
}

def video_streamer_thread(client):
    """
    This function runs in a separate thread. It continuously captures images
    from AirSim, encodes them as JPEGs, and sends them over a UDP socket.
    """
    # Setup UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (UDP_IP, UDP_PORT)
    print(f"🚀 Video streaming thread started. Sending JPEG frames to {UDP_IP}:{UDP_PORT}.")
    
    request = airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False)

    while ptz_state['running']:
        start_time = time.time()
        
        responses = client.simGetImages([request])
        response = responses[0]

        if response.image_data_uint8:
            img_1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img_1d.reshape(response.height, response.width, 3)
            
            if response.width != IMG_WIDTH or response.height != IMG_HEIGHT:
                img_bgr = cv2.resize(img_bgr, (IMG_WIDTH, IMG_HEIGHT))

            # Encode the frame as JPEG
            result, encoded_frame = cv2.imencode('.jpg', img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            
            if result:
                try:
                    # Send the JPEG data over UDP
                    udp_socket.sendto(encoded_frame.tobytes(), server_address)
                except Exception as e:
                    # This can happen if the frame is too large for a single UDP packet
                    print(f"Warning: Could not send frame. Error: {e}")
                    # For a robust solution, frames would need to be chunked.

        elapsed_time = time.time() - start_time
        sleep_time = (1.0 / FPS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    print("🛑 Video streaming thread stopped.")
    udp_socket.close()

def ptz_controller_thread():
    """
    This function runs in a separate thread. It listens for keyboard inputs
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
        if keyboard.is_pressed('a') or keyboard.is_pressed('left'): ptz_state['pan'] -= PAN_SPEED
        if keyboard.is_pressed('d') or keyboard.is_pressed('right'): ptz_state['pan'] += PAN_SPEED
        if keyboard.is_pressed('w') or keyboard.is_pressed('up'): ptz_state['tilt'] -= TILT_SPEED
        if keyboard.is_pressed('s') or keyboard.is_pressed('down'): ptz_state['tilt'] += TILT_SPEED
        if keyboard.is_pressed('+') or keyboard.is_pressed('='): ptz_state['fov'] -= ZOOM_SPEED
        if keyboard.is_pressed('-'): ptz_state['fov'] += ZOOM_SPEED
        if keyboard.is_pressed('esc'):
            ptz_state['running'] = False
            print("\nESC pressed. Shutting down...")
            
        ptz_state['tilt'] = max(-89.0, min(89.0, ptz_state['tilt']))
        ptz_state['fov'] = max(MIN_FOV, min(MAX_FOV, ptz_state['fov']))
        ptz_state['pan'] %= 360

        time.sleep(0.05)

def main():
    """
    Main function to initialize AirSim client, start threads,
    and apply PTZ updates.
    """
    # --- Initialize AirSim Client ---
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        client.enableApiControl(True, VEHICLE_NAME)
        print("✅ Successfully connected to AirSim and enabled API control.")
    except Exception as e:
        print(f"Error: Could not connect to AirSim. Please ensure it is running.")
        print(f"Details: {e}")
        return

    # --- Start Threads ---
    stream_thread = threading.Thread(target=video_streamer_thread, args=(client,))
    controller_thread = threading.Thread(target=ptz_controller_thread)
    
    stream_thread.start()
    controller_thread.start()

    # --- Main loop to apply PTZ updates ---
    last_pan, last_tilt, last_fov = -1, -1, -1
    while ptz_state['running']:
        pan, tilt, fov = ptz_state['pan'], ptz_state['tilt'], ptz_state['fov']
        
        # *** FIX for AttributeError ***
        # The correct API is simSetCameraPose, not simSetCameraOrientation.
        if pan != last_pan or tilt != last_tilt:
            # Get the current camera pose to preserve its position
            current_pose = client.simGetCameraPose(CAMERA_NAME, vehicle_name=VEHICLE_NAME)
            
            # Create a new orientation quaternion from our PTZ state
            q = airsim.to_quaternion(math.radians(tilt), 0, math.radians(pan))
            
            # Update the orientation part of the pose, keeping the position the same
            current_pose.orientation = q
            
            # Set the new pose in the simulator
            client.simSetCameraPose(CAMERA_NAME, current_pose, vehicle_name=VEHICLE_NAME)
            
            last_pan, last_tilt = pan, tilt
            
        if fov != last_fov:
            client.simSetCameraFov(CAMERA_NAME, fov, vehicle_name=VEHICLE_NAME)
            last_fov = fov
            
        time.sleep(0.02)

    # --- Cleanup ---
    print("Waiting for threads to join...")
    stream_thread.join()
    controller_thread.join()
    client.enableApiControl(False, VEHICLE_NAME)
    print("✅ Application has been shut down gracefully.")

if __name__ == "__main__":
    main()
