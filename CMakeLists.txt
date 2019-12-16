
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

### Project Configuration

SET(PROJECT_NAME "detection3D")
PROJECT("${PROJECT_NAME}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Resource and Shader Paths ###

# Image input folder
SET(INPUT_DIR "${CMAKE_SOURCE_DIR}/input")

# Shader folder
SET(SHADER_DIR "${CMAKE_SOURCE_DIR}/shaders")

### External Dependencies ###

# OpenCV (2.4)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# OpenGL
SET(OpenGL_GL_PREFERENCE "LEGACY") # LEGACY or GLVND
FIND_PACKAGE(OpenGL REQUIRED)

# GLEW
FIND_PACKAGE(GLEW REQUIRED)

# GLFW (No find package available with cmake 3.13, so it is hard coded to point to system lib)
SET(GLFW_LIBRARIES /usr/lib/x86_64-linux-gnu/libglfw.so)
SET(GLFW_INCLUDE_DIRS /usr/include)

SET(EXTERNAL_LIBS
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARY}
        ${GLEW_LIBRARIES}
        ${GLFW_LIBRARIES})

SET(EXTERNAL_INCLUDE
        ${OpenCV_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS})

### Internal Libraries ###

SUBDIRS(vivalib detectionlib)

### Source Files and resources ###
FILE(GLOB files
        "*.h"
        "*.cpp"
        "ObjectRotator/*.h"
        "ObjectRotator/*.cpp"
        "detectionlib/*.h"
        "detectionlib/*.cpp"
        )

FILE(GLOB hidden
        ".*"
        )

FILE(GLOB resources
        "*.*"
        )
LIST(REMOVE_ITEM resources ${files} ${hidden} "${CMAKE_SOURCE_DIR}/CMakeLists.txt" "${CMAKE_SOURCE_DIR}/README.md")

### Status Report ###

MESSAGE(STATUS "Project Name: ${PROJECT_NAME}")
MESSAGE(STATUS "    OpenCV version: ${OpenCV_VERSION} ")
MESSAGE(STATUS "    OpenCV libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    OpenCV include path: ${OpenCV_INCLUDE_DIRS}")

MESSAGE(STATUS "")

MESSAGE(STATUS "    OpenGL libraries: ${OPENGL_LIBRARY}")
MESSAGE(STATUS "    OpenGL include path: ${OPENGL_INCLUDE_DIR}")


MESSAGE(STATUS "")

MESSAGE(STATUS "    GLEW libraries: ${GLEW_LIBRARIES}")
MESSAGE(STATUS "    GLEW include path: ${GLEW_INCLUDE_DIRS}")

MESSAGE(STATUS "")

MESSAGE(STATUS "    GLFW libraries: ${GLFW_LIBRARIES}")
MESSAGE(STATUS "    GLFW include path: ${GLFW_INCLUDE_DIRS}")

MESSAGE(STATUS "")

### Target Definition ###

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/vivalib ${CMAKE_SOURCE_DIR}/detectionlib)
ADD_EXECUTABLE(${PROJECT_NAME} ${files} ObjectRotator/cvRANSAC.h ObjectRotator/EMEstimator.cpp ObjectRotator/cvRANSAC.cpp ObjectRotator/extractRotationMatrix.h ObjectRotator/extractRotationMatrix.cpp ObjectRotator/cameraCalibration.h ObjectRotator/cameraCalibration.cpp ObjectRotator/kalmanForRotMat.h ObjectRotator/kalmanForRotMat.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${EXTERNAL_LIBS} vivalib detectionlib)

### Copy Resources ###

FILE(WRITE ${CMAKE_SOURCE_DIR}/input.txt "${CMAKE_SOURCE_DIR}/input/")

FILE(GLOB inputs ${CMAKE_SOURCE_DIR}/input/*)
FILE(COPY ${inputs} DESTINATION "input")

FILE(GLOB shaders ${CMAKE_SOURCE_DIR}/shaders/*)
FILE(COPY ${shaders} DESTINATION "shaders")