cmake_minimum_required(VERSION 2.8)

project(simple-opencv-kalman-tracker)

set(CMAKE_BUILD_TYPE Debug)

find_package( OpenCV REQUIRED )



set(SRC_PATH source )

set(${PROJECT_NAME}_SRC
   ${SRC_PATH}/opencv-kalman.cpp
)


#########################################################
# Executable
add_executable( ${PROJECT_NAME} ${${PROJECT_NAME}_SRC} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
