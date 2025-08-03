
# AirSim PTZ Camera Simulator (Local Display & cv2.waitKey)
#
# This script simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
# It displays the video feed locally in an OpenCV window and uses cv2.waitKey()
# to capture keyboard input for PTZ controls.
#
# This version does NOT stream video over the network and does NOT require
# a separate client script or the pynput/keyboard libraries.
#
# Author: Gemini
# Date: August 2, 2025

# --- Installation ---
# pip install airsim opencv-python numpy

# --- How to Run ---
# 1. Start your AirSim environment with PX4 SITL.
# 2. Run this Python script.
# 3. A window will appear showing the camera feed.
# 4. Make sure the video window is in focus to use the PTZ controls.

import airsim
import cv2
import numpy as np
import time
import math

# --- Configuration ---
# Display Configuration
IMG_WIDTH = 1280          # Video frame width
IMG_HEIGHT = 720          # Video frame height
FPS = 30                  # Frames per second

# PTZ Control Configuration
# This name MUST match the vehicle name in your settings.json
VEHICLE_NAME = "PX4" 
CAMERA_NAME = "0"       # "0" is the front-center camera
PAN_SPEED = 2.0         # Degrees per step for pan
TILT_SPEED = 2.0        # Degrees per step for tilt
ZOOM_SPEED = 2.0        # Degrees per step for zoom (FoV change)
MIN_FOV = 0.15           # Minimum Field of View (max zoom)
MAX_FOV = 60            # Maximum Field of View (min zoom)

def main():
    """
    Main function to initialize AirSim, display video, and handle all API calls.
    """
    # --- Initialize AirSim Client ---
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        # *** FIX for PX4: Do not enable API control for flight. ***
        # The PX4 firmware is in control of flight, but we can still control the camera.
        # client.enableApiControl(True, VEHICLE_NAME) 
        print("✅ Successfully connected to AirSim.")
    except Exception as e:
        print(f"Error: Could not connect to AirSim. Please ensure it is running.")
        print(f"Details: {e}")
        return

    print("\n--- PTZ Camera Controls ---")
    print("  Pan Left:    'a'")
    print("  Pan Right:   'd'")
    print("  Tilt Up:     'w'")
    print("  Tilt Down:   's'")
    print("  Zoom In:     'e'")
    print("  Zoom Out:    'q'")
    print("  Exit:        'esc'")
    print("---------------------------\n")
    print("ℹ️  Focus the 'AirSim Feed' window to use controls.")

    request = airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False)

    # --- PTZ State Variables ---
    pan = 0.0
    tilt = 0.0
    fov = MAX_FOV
    running = True

    # --- Main Loop for Video, Input, and API Calls ---
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
            
            # Display the image in a window
            cv2.imshow("AirSim Feed", img_bgr)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF

        # Update PTZ state based on input
        if key == ord('a'): pan -= PAN_SPEED
        elif key == ord('d'): pan += PAN_SPEED
        elif key == ord('w'): tilt -= TILT_SPEED
        elif key == ord('s'): tilt += TILT_SPEED
        elif key == ord('e'): fov -= ZOOM_SPEED
        elif key == ord('q'): fov += ZOOM_SPEED
        elif key == 27: # 27 is the ASCII code for the Escape key
            print("\nESC pressed. Shutting down...")
            running = False
            break

        # Clamp values
        tilt = max(-89.0, min(89.0, tilt))
        fov = max(MIN_FOV, min(MAX_FOV, fov))
        pan %= 360
        
        # --- Apply PTZ updates directly in the main loop ---
        try:
            # Set camera orientation
            current_pose = client.simGetCameraInfo(CAMERA_NAME, vehicle_name=VEHICLE_NAME).pose
            q = airsim.to_quaternion(math.radians(tilt), 0, math.radians(pan))
            current_pose.orientation = q
            current_pose.position.x_val = 0
            current_pose.position.y_val = 0
            current_pose.position.z_val = 0
            client.simSetCameraPose(CAMERA_NAME, current_pose, vehicle_name=VEHICLE_NAME)

            # Set camera field of view (zoom)
            client.simSetCameraFov(CAMERA_NAME, fov, vehicle_name=VEHICLE_NAME)
        except Exception as e:
            if running:
                print(f"Error updating camera: {e}")

        # Regulate the loop to approximate the desired FPS
        elapsed_time = time.time() - start_time
        sleep_time = (1.0 / FPS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- Cleanup ---
    cv2.destroyAllWindows()
    # We never enabled API control, so no need to disable it.
    # client.enableApiControl(False, VEHICLE_NAME)
    print("✅ Application has been shut down gracefully.")

if __name__ == "__main__":
    main()
