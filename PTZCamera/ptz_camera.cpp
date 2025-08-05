// AirSim PTZ Camera Simulator (C++)
//
// This application simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
// It uses OpenCV's VideoWriter with a GStreamer backend to encode and stream video.
//
// It also displays the feed locally in an OpenCV window to capture keyboard input.
//
// Author: Gemini
// Date: August 3, 2025

// --- Dependencies ---
// - AirSim C++ Client (msrpc-c_cxx)
// - OpenCV 4+ (MUST be compiled with GStreamer support)
// - GStreamer 1.0+ (runtime libraries)

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

// Define an alias for the exception type to simplify the catch blocks.
using AirSimRpcException = rpc::rpc_error;

int main()
{
    // --- GStreamer Pipeline for OpenCV VideoWriter ---
    // This pipeline string tells OpenCV to take frames from the application (appsrc),
    // convert their color space, encode them to H.264 (x264enc),
    // package them for streaming (rtph264pay), and send them via UDP (udpsink).
    std::string gst_pipeline_str =
        "appsrc ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=4000 speed-preset=fast ! "
        "rtph264pay ! "
        "udpsink host=" +
        UDP_IP + " port=" + std::to_string(UDP_PORT) + " sync=false";

    // --- Create OpenCV GStreamer VideoWriter ---
    cv::VideoWriter video_writer;
    video_writer.open(gst_pipeline_str, cv::CAP_GSTREAMER, 0, FPS, cv::Size(IMG_WIDTH, IMG_HEIGHT), true);
    if (!video_writer.isOpened()) {
        std::cerr << "Error: Failed to open GStreamer VideoWriter." << std::endl;
        std::cerr << "Please ensure your OpenCV installation has GStreamer support." << std::endl;
        return -1;
    }
    std::cout << "✅ GStreamer VideoWriter opened successfully." << std::endl;
    std::cout << "Streaming to udp://" << UDP_IP << ":" << UDP_PORT << std::endl;

    // --- Initialize AirSim Client ---
    msr::airlib::MultirotorRpcLibClient client;
    try {
        client.confirmConnection();
        std::cout << "✅ Successfully connected to AirSim." << std::endl;
    }
    catch (AirSimRpcException& e) {
        std::cerr << "Error: Could not connect to AirSim. Please ensure it is running." << std::endl;
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
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
    std::cout << "---------------------------\n"
              << std::endl;
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
                0, // roll
                msr::airlib::Utils::degreesToRadians(pan) // yaw
            );
            msr::airlib::Pose new_camera_pose(Eigen::Vector3f(0, 0, 0), orientation_quat);
            client.simSetCameraPose(CAMERA_NAME, new_camera_pose, VEHICLE_NAME);
            client.simSetCameraFov(CAMERA_NAME, fov, VEHICLE_NAME);
        }
        catch (AirSimRpcException& e) {
            if (running) {
                std::cerr << "Error updating camera: " << e.get_error().as<std::string>() << std::endl;
            }
        }

        // --- Image Capture and Streaming ---
        const std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
            msr::airlib::ImageCaptureBase::ImageRequest(CAMERA_NAME, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
        };
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& response = client.simGetImages(request, VEHICLE_NAME);

        if (!response.empty() && !response[0].image_data_uint8.empty()) {
            // The API returns raw BGR data, not a compressed JPEG.
            // We must construct the cv::Mat directly from the raw data vector.
            cv::Mat img_bgr(response[0].height, response[0].width, CV_8UC3, (void*)response[0].image_data_uint8.data());

            if (!img_bgr.empty()) {
                cv::Mat resized_img;
                if (img_bgr.cols != IMG_WIDTH || img_bgr.rows != IMG_HEIGHT) {
                    cv::resize(img_bgr, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT));
                }
                else {
                    resized_img = img_bgr.clone(); // Clone to ensure data is copied
                }

                // Display locally to capture key presses
                cv::imshow("AirSim C++ Feed", resized_img);

                // Write frame to the GStreamer pipeline via OpenCV
                video_writer.write(resized_img);
            }
        }

        // --- Keyboard Input ---
        int key = cv::waitKey(1) & 0xFF;
        switch (key) {
        case 'a':
            pan -= PAN_SPEED;
            break;
        case 'd':
            pan += PAN_SPEED;
            break;
        case 'w':
            tilt -= TILT_SPEED;
            break;
        case 's':
            tilt += TILT_SPEED;
            break;
        case 'e':
            fov -= ZOOM_SPEED;
            break;
        case 'q':
            fov += ZOOM_SPEED;
            break;
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
    video_writer.release();
    cv::destroyAllWindows();
    std::cout << "✅ Application has been shut down gracefully." << std::endl;

    return 0;
}
