// AirSim PTZ Camera Simulator (C++ - Refactored for Thread Safety)
//
// This application simulates a Pan-Tilt-Zoom (PTZ) camera using Microsoft AirSim.
// It uses a thread-safe wrapper for AirSim calls and separates PTZ control
// into a background thread to prevent video stutter.
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
#include <mutex>
#include <atomic>

// AirSim C++ Client
#include "rpc/rpc_error.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// OpenCV
#include "opencv2/opencv.hpp"

// --- Configuration ---
const std::string UDP_IP = "127.0.0.1";
const int UDP_PORT = 5000;
const int IMG_WIDTH = 1280;
const int IMG_HEIGHT = 720;
const int FPS = 30;
const std::string VEHICLE_NAME = "PX4";
const std::string CAMERA_NAME = "0";
const float PAN_SPEED = 2.0f;
const float TILT_SPEED = 2.0f;
const float ZOOM_SPEED = 2.0f;
const float MIN_FOV = 15.0f;
const float MAX_FOV = 90.0f;

using AirSimRpcException = rpc::rpc_error;

// --- Thread-Safe AirSim Wrapper ---
class AirsimWrapper {
public:
    AirsimWrapper() {}

    bool connect() {
        try {
            client_.confirmConnection();
            std::cout << "✅ Successfully connected to AirSim." << std::endl;
            return true;
        } catch (AirSimRpcException& e) {
            std::cerr << "Error: Could not connect to AirSim." << std::endl;
            std::cerr << "Details: " << e.get_error().as<std::string>() << std::endl;
            return false;
        }
    }

    void updatePtz(float pan, float tilt, float fov) {
        std::lock_guard<std::mutex> lock(client_mutex_);
        try {
            msr::airlib::Pose vehicle_pose = client_.simGetVehiclePose(VEHICLE_NAME);
            msr::airlib::Quaternionr orientation_quat = msr::airlib::VectorMath::toQuaternion(
                msr::airlib::Utils::degreesToRadians(tilt), 0, msr::airlib::Utils::degreesToRadians(pan));
            
            // The camera's position is relative to the vehicle's frame, so (0,0,0) is correct here.
            msr::airlib::Pose new_camera_pose(Eigen::Vector3f(0, 0, 0), orientation_quat);
            
            client_.simSetCameraPose(CAMERA_NAME, new_camera_pose, VEHICLE_NAME);
            client_.simSetCameraFov(CAMERA_NAME, fov, VEHICLE_NAME);
        } catch (AirSimRpcException& e) {
            std::cerr << "Error updating camera: " << e.get_error().as<std::string>() << std::endl;
        }
    }

    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> getImages() {
        std::lock_guard<std::mutex> lock(client_mutex_);
        const std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
            msr::airlib::ImageCaptureBase::ImageRequest(CAMERA_NAME, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
        };
        return client_.simGetImages(request, VEHICLE_NAME);
    }

private:
    msr::airlib::MultirotorRpcLibClient client_;
    std::mutex client_mutex_;
};

// --- Shared State for Threads ---
std::atomic<bool> running(true);
std::atomic<float> pan(0.0f);
std::atomic<float> tilt(0.0f);
std::atomic<float> fov(MAX_FOV);

// --- PTZ Control Thread ---
// This thread sends PTZ updates to AirSim in the background.
void ptz_control_thread(AirsimWrapper& airsim_wrapper) {
    while (running) {
        // Read the latest atomic values
        float current_pan = pan.load();
        float current_tilt = tilt.load();
        float current_fov = fov.load();

        airsim_wrapper.updatePtz(current_pan, current_tilt, current_fov);
        
        // Send updates at a reasonable rate
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// --- Main Thread (Image Handling and Keyboard Input) ---
int main() {
    // --- GStreamer Pipeline Setup ---
    std::string gst_pipeline_str =
        "appsrc ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=3000 speed-preset=fast ! "
        "rtph264pay ! "
        "udpsink host=" + UDP_IP + " port=" + std::to_string(UDP_PORT) + " sync=false buffer-size=1000000";

    cv::VideoWriter video_writer;
    video_writer.open(gst_pipeline_str, cv::CAP_GSTREAMER, 0, FPS, cv::Size(IMG_WIDTH, IMG_HEIGHT), true);
    if (!video_writer.isOpened()) {
        std::cerr << "Error: Failed to open GStreamer VideoWriter." << std::endl;
        return -1;
    }
    std::cout << "✅ GStreamer VideoWriter opened successfully." << std::endl;
    std::cout << "Streaming to udp://" << UDP_IP << ":" << UDP_PORT << std::endl;

    // --- AirSim Setup ---
    AirsimWrapper airsim_wrapper;
    if (!airsim_wrapper.connect()) {
        return -1;
    }
    
    // --- Start Background Thread for PTZ Control ---
    std::thread ptz_thread(ptz_control_thread, std::ref(airsim_wrapper));

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

    // --- Main Loop for Image Acquisition, Display, and Input ---
    while (running) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Get image from AirSim
        auto response = airsim_wrapper.getImages();

        if (!response.empty() && !response[0].image_data_uint8.empty()) {
            cv::Mat img_bgr(response[0].height, response[0].width, CV_8UC3, (void*)response[0].image_data_uint8.data());
            if (!img_bgr.empty()) {
                cv::Mat resized_img;
                cv::resize(img_bgr, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT));
                
                cv::imshow("AirSim C++ Feed", resized_img);
                video_writer.write(resized_img);
            }
        }

        // Handle keyboard input
        int key = cv::waitKey(1) & 0xFF;
        float current_pan = pan.load();
        float current_tilt = tilt.load();
        float current_fov = fov.load();

        switch (key) {
            case 'a': current_pan -= PAN_SPEED; break;
            case 'd': current_pan += PAN_SPEED; break;
            case 'w': current_tilt -= TILT_SPEED; break;
            case 's': current_tilt += TILT_SPEED; break;
            case 'e': current_fov -= ZOOM_SPEED; break;
            case 'q': current_fov += ZOOM_SPEED; break;
            case 27: running = false; break;
        }

        // Clamp and update atomic variables
        pan.store(fmod(current_pan, 360.0f));
        tilt.store(std::max(-89.0f, std::min(89.0f, current_tilt)));
        fov.store(std::max(MIN_FOV, std::min(MAX_FOV, current_fov)));

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
    if (ptz_thread.joinable()) {
        ptz_thread.join();
    }
    video_writer.release();
    cv::destroyAllWindows();
    std::cout << "✅ Application has been shut down gracefully." << std::endl;

    return 0;
}
