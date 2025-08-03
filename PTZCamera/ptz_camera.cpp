// AirSim PTZ Camera Simulator (C++)
//
// This application simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
// It launches a GStreamer pipeline as a separate process and pipes raw video
// frames to it for H.264 encoding and UDP streaming.
//
// It also displays the feed locally in an OpenCV window to capture keyboard input.
//
// Author: Gemini
// Date: August 3, 2025

// --- Dependencies ---
// - AirSim C++ Client (msrpc-c_cxx)
// - OpenCV 4+
// - GStreamer 1.0+ (must be in system PATH)

// --- How to Build ---
// 1. Make sure all dependencies are installed.
// 2. Create a 'build' directory: mkdir build && cd build
// 3. Run CMake: cmake ..
// 4. Compile: make (or open the generated solution in Visual Studio and build)

// --- How to Run ---
// 1. Start your AirSim environment with PX4 SITL.
// 2. Run the compiled executable from your build directory.
// 3. Open the stream in VLC using the 'stream.sdp' file.
// 4. Make sure the 'AirSim C++ Feed' window is in focus to use PTZ controls.

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "rpc/rpc_error.h"
// AirSim C++ Client
// #include "vehicles/multirotor/api/MultirotorRpcClient.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// OpenCV
#include "opencv2/opencv.hpp"

// --- Configuration ---
// Streaming
const std::string UDP_IP = "127.0.0.1";
const int UDP_PORT = 5000;
const int IMG_WIDTH = 1280;
const int IMG_HEIGHT = 720;
const int FPS = 30;

// PTZ Control
const std::string VEHICLE_NAME = "PX4";
const std::string CAMERA_NAME = "0";
const float PAN_SPEED = 2.0f;
const float TILT_SPEED = 2.0f;
const float ZOOM_SPEED = 2.0f;
const float MIN_FOV = 15.0f;
const float MAX_FOV = 90.0f;

#define POPEN popen
#define PCLOSE pclose

int main() {
    // --- GStreamer Pipeline Command ---
    std::string gst_command =
        "gst-launch-1.0 -v fdsrc ! "
        "videoparse format=bgr width=" + std::to_string(IMG_WIDTH) +
        " height=" + std::to_string(IMG_HEIGHT) + " framerate=" + std::to_string(FPS) + "/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=4000 speed-preset=superfast ! "
        "rtph264pay ! "
        "udpsink host=" + UDP_IP + " port=" + std::to_string(UDP_PORT) + " sync=false";

    // --- Launch GStreamer as a Subprocess ---
    std::cout << "🚀 Launching GStreamer pipeline..." << std::endl;
    std::cout << "Streaming to udp://" << UDP_IP << ":" << UDP_PORT << std::endl;
    FILE* gst_pipe = POPEN(gst_command.c_str(), "w");
    if (!gst_pipe) {
        std::cerr << "Error: Failed to launch GStreamer. Is it in your system's PATH?" << std::endl;
        return -1;
    }

    // --- Initialize AirSim Client ---
    msr::airlib::MultirotorRpcLibClient client;
    try {
        client.confirmConnection();
        std::cout << "✅ Successfully connected to AirSim." << std::endl;
    } catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
        // std::cerr << "Error: Could not connect to AirSim. Please ensure it is running." << std::endl;
        // std::cerr << "Details: " << e.get_error().as<std::string>() << std::endl;
        PCLOSE(gst_pipe);
        return -1;
    }

    std::cout << "\n--- PTZ Camera Controls ---" << std::endl;
    std::cout << "  Pan Left:    'a'" << std::endl;
    std::cout << "  Pan Right:   'd'" << std::endl;
    std::cout << "  Tilt Up:     'w'" << std::endl;
    std::cout << "  Tilt Down:   's'" << std::endl;
    std::cout << "  Zoom In:     'e'" << std::endl;
    std::cout << "  Zoom Out:    'q'" << std::endl;
    std::cout << "  Exit:        'esc'" << std::endl;
    std::cout << "---------------------------\n" << std::endl;
    std::cout << "ℹ️  Focus the 'AirSim C++ Feed' window to use controls." << std::endl;

    // --- PTZ State Variables ---
    float pan = 0.0f;
    float tilt = 0.0f;
    float fov = MAX_FOV;
    bool running = true;

    // --- Main Loop ---
    while (running) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // --- PTZ Camera Control ---
        try {
            msr::airlib::Pose vehicle_pose = client.simGetVehiclePose(VEHICLE_NAME);
            msr::airlib::Quaternionr orientation_quat = msr::airlib::VectorMath::toQuaternion(
                msr::airlib::Utils::degreesToRadians(tilt), // pitch
                0,                                          // roll
                msr::airlib::Utils::degreesToRadians(pan)   // yaw
            );
            msr::airlib::Pose new_camera_pose(vehicle_pose.position, orientation_quat);
            client.simSetCameraPose(CAMERA_NAME, new_camera_pose, VEHICLE_NAME);
            client.simSetCameraFov(CAMERA_NAME, fov, VEHICLE_NAME);
        } catch (rpc::rpc_error& e) {
            if (running) {
              const auto msg = e.get_error().as<std::string>();
              std::cout << "Exception raised by the API, something went wrong." << std::endl
                        << msg << std::endl;
            }
        }

        // --- Image Capture and Streaming ---
        const std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
            msr::airlib::ImageCaptureBase::ImageRequest(CAMERA_NAME, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
        };
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& response = client.simGetImages(request, VEHICLE_NAME);

        if (!response.empty() && !response[0].image_data_uint8.empty()) {
            // Convert to OpenCV Mat
            cv::Mat img_bgr = cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);
            
            if (!img_bgr.empty()) {
                cv::Mat resized_img;
                if (img_bgr.cols != IMG_WIDTH || img_bgr.rows != IMG_HEIGHT) {
                    cv::resize(img_bgr, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT));
                } else {
                    resized_img = img_bgr;
                }

                // Display locally to capture key presses
                cv::imshow("AirSim C++ Feed", resized_img);

                // Write raw frame data to the GStreamer process
                fwrite(resized_img.data, 1, resized_img.total() * resized_img.elemSize(), gst_pipe);
            }
        }

        // --- Keyboard Input ---
        int key = cv::waitKey(1) & 0xFF;
        switch (key) {
            case 'a': pan -= PAN_SPEED; break;
            case 'd': pan += PAN_SPEED; break;
            case 'w': tilt -= TILT_SPEED; break;
            case 's': tilt += TILT_SPEED; break;
            case 'e': fov -= ZOOM_SPEED; break;
            case 'q': fov += ZOOM_SPEED; break;
            case 27: // ESC key
                running = false;
                break;
        }

        // Clamp values
        tilt = std::max(-89.0f, std::min(89.0f, tilt));
        fov = std::max(MIN_FOV, std::min(MAX_FOV, fov));
        pan = fmod(pan, 360.0f);

        // Regulate loop speed
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        long sleep_ms = (1000 / FPS) - duration.count();
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    // --- Cleanup ---
    std::cout << "Cleaning up resources..." << std::endl;
    PCLOSE(gst_pipe);
    cv::destroyAllWindows();
    std::cout << "✅ Application has been shut down gracefully." << std::endl;

    return 0;
}
