
cmake_minimum_required(VERSION 3.5.0)
project(PTZCamera)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find OpenCV ---
# OpenCV is used for image processing and creating a display window.
find_package(OpenCV REQUIRED)

IncludeEigen()

SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloDrone
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
  # ${AIRSIM_ROOT}/MavLinkCom/include
  # ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
