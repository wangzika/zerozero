cmake_minimum_required(VERSION 3.16)
project(stream_monitor)

## Compile as C++17 (required by Rerun)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

## Find required packages
find_package(OpenCV REQUIRED)
set(Protobuf_PROTOC_EXECUTABLE /usr/local/bin/protoc)
find_package(Protobuf REQUIRED)
# find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nanomsg REQUIRED)
# find_package(nanomsg CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

include(cmake/GenerateProto.cmake)
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/_deps)

# Find or fetch Rerun C++ SDK
set(RERUN_CPP_URL "https://github.com/rerun-io/rerun/releases/download/0.22.0/rerun_cpp_sdk.zip" CACHE STRING "URL to the rerun_cpp zip.")
message(STATUS "RERUN_CPP_URL = ${RERUN_CPP_URL}")

include(FetchContent)
FetchContent_Declare(rerun_sdk 
URL ${RERUN_CPP_URL}
)
FetchContent_MakeAvailable(rerun_sdk)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${NANOMSG_INCLUDE_DIRS}
)

## Generate protobuf files
set(PROTO_FILES
  proto/captain_msgs.proto
  proto/replaykit.proto
  proto/rovio.proto
  proto/image.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})


add_library(loopClosing_lib STATIC
    loopClosing/src/loop_closure_detector.cpp
    loopClosing/src/VocabularyBinary.cpp
    loopClosing/src/BowVector.cpp
    loopClosing/src/FBrief.cpp
    loopClosing/src/FeatureVector.cpp
    loopClosing/src/QueryResults.cpp
    loopClosing/src/ScoringObject.cpp
    loopClosing/src/Random.cpp
    loopClosing/src/Timestamp.cpp
    loopClosing/src/BRIEF.cpp
    loopClosing/src/utility.cpp
    loopClosing/src/keyframe.cpp
    loopClosing/src/beblid.cpp
    loopClosing/src/posegraph.cpp
    # src/zzTimer.cpp

)


target_include_directories(loopClosing_lib PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    loopClosing/include
)

target_link_libraries(loopClosing_lib PUBLIC
    ${OpenCV_LIBS}
    rerun_sdk
)
## Add protobuf generated files to include directories
include_directories(${CMAKE_CURRENT_BINARY_DIR})


add_subdirectory(replaykit)



## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/network_manager.cpp
  ${PROTO_SRCS}
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  replaykit/include
)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${OpenCV_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${NANOMSG_LIBRARIES}
  replaykit
  nanomsg
  rerun_sdk

)

## Declare a C++ executable
add_executable(${PROJECT_NAME} main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_lib)

add_executable(data_collector datacollector.cpp)
add_dependencies(data_collector ${PROJECT_NAME}_lib)

# add_executable(rerun rerun.cpp)
# add_dependencies(rerun ${PROJECT_NAME}_lib loopClosing_lib)


add_executable(rerun1 rerun1.cpp src/rerun_loop.cpp)
add_dependencies(rerun1 ${PROJECT_NAME}_lib loopClosing_lib)



# target_include_directories(rerun PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/loopClosing/include
# )
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${NANOMSG_LIBRARIES}
  nanomsg
  rerun_sdk
)

target_link_libraries(data_collector PRIVATE
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${NANOMSG_LIBRARIES}
  nanomsg
  rerun_sdk
)

# target_link_libraries(rerun PRIVATE
#   ${PROJECT_NAME}_lib
#   ${OpenCV_LIBRARIES}
#   ${Protobuf_LIBRARIES}
#   ${NANOMSG_LIBRARIES}
#   # ${ZZLOG_INCLUDE_DIR}
#   loopClosing_lib
#   nanomsg
#   rerun_sdk
# )


# # --- 修改 rerun 的链接 ---
# target_link_libraries(rerun PRIVATE
#   ${PROJECT_NAME}_lib
#   ${OpenCV_LIBRARIES}
#   ${Protobuf_LIBRARIES}
#   ${NANOMSG_LIBRARIES}
#   loopClosing_lib
#   nanomsg
#   rerun_sdk
# )


# --- 修改 rerun 的链接 ---
target_link_libraries(rerun1 PRIVATE
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${NANOMSG_LIBRARIES}
  loopClosing_lib
  nanomsg
  rerun_sdk
)