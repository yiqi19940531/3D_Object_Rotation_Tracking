//
// Created by dlalancette on 19/02/19.
//
#include "ObjectRotator.h"

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glm/gtx/string_cast.hpp>

static const glm::vec3 xAxis = glm::vec3(1.0f, 0.0f, 0.0f);
static const glm::vec3 yAxis = glm::vec3(0.0f, 1.0f, 0.0f);
static const glm::vec3 zAxis = glm::vec3(0.0f, 0.0f, 1.0f);


void ObjectRotator::renderCube()
{
    interpolationParam_x = interpolationParam_x + 0.1f > 1.0f ? 1.0f : interpolationParam_x + 0.1f;
    currentAngle_x = interpolationParam_x * desiredAngle_x + (1 - interpolationParam_x) * currentAngle_x;

    interpolationParam_y = interpolationParam_y + 0.1f > 1.0f ? 1.0f : interpolationParam_y + 0.1f;
    currentAngle_y = interpolationParam_y * desiredAngle_y + (1 - interpolationParam_y) * currentAngle_y;

    interpolationParam_z = interpolationParam_z + 0.1f > 1.0f ? 1.0f : interpolationParam_z + 0.1f;
    currentAngle_z = interpolationParam_z * desiredAngle_z + (1 - interpolationParam_z) * currentAngle_z;

//    currentAngle_y = desiredAngle_y;
//    currentAngle_z = desiredAngle_z;

    auto transform = glm::mat4(1.0f);

    transform = glm::rotate(transform, (glm::mediump_float)currentAngle_z, zAxis);
    //transform = glm::rotate(transform, (glm::mediump_float)currentAngle_y, yAxis);
    //transform = glm::rotate(transform, (glm::mediump_float)currentAngle_x, xAxis);

    render(transform);
}

void ObjectRotator::setRotation(float radians)
{
    currentAngle_x = radians;
    currentAngle_y = radians;
    currentAngle_z = radians;
}

void ObjectRotator::setRotation(const glm::vec3 &normal, float radians)
{
    interpolationParam_y = 0.0f;

    /// The magic number "1.8f" is there to compensate for the lack of accuracy of the normal calculation.
    /// It works quite well to make the cube follow the real world rotation more closely. It can only achieve about 50%
    /// of the angles of the physical object otherwise.
    float angleRad = 1.8f * glm::sign(normal.x) * glm::atan(abs(normal.x) / normal.z);

    desiredAngle_y = -radians + angleRad;

    float directDifference = abs(desiredAngle_y - currentAngle_y);
    float wrapDifference = abs((desiredAngle_y + 2 * PI) - currentAngle_y);

    if (directDifference  > wrapDifference)
    {
        desiredAngle_y += + PI * 2;
    }
}

void ObjectRotator::setRotation_x(const float xAxisRadians, float face_radians)
{
    interpolationParam_x = 0.0f;
    desiredAngle_x = -face_radians + xAxisRadians;
//    std::cout<<xAxisRadians<<std::endl;

    float directDifference = abs(desiredAngle_x - currentAngle_x);
    float wrapDifference = abs((desiredAngle_x + 2 * PI) - currentAngle_x);

    if (directDifference  > wrapDifference)
    {
        desiredAngle_x += + PI * 2;
    }
}

void ObjectRotator::setRotation_y(const float yAxisRadians, float face_radians)
{
    interpolationParam_y = 0.0f;
    desiredAngle_y = -face_radians + yAxisRadians;
    std::cout<<yAxisRadians<<std::endl;
//    std::cout<<-face_radians<<std::endl;

    float directDifference = abs(desiredAngle_y - currentAngle_y);
    float wrapDifference = abs((desiredAngle_y + 2 * PI) - currentAngle_y);

    if (directDifference  > wrapDifference)
    {
        desiredAngle_y += + PI * 2;
    }
}

void ObjectRotator::setRotation_z(const float zAxisRadians)
{
    interpolationParam_z = 0.0f;
    desiredAngle_z = zAxisRadians;

    std::cout<<zAxisRadians<<std::endl;

    float directDifference = abs(desiredAngle_z - currentAngle_z);
    float wrapDifference = abs((desiredAngle_z + 2 * PI) - currentAngle_z);

    if (directDifference  > wrapDifference)
    {
        desiredAngle_z += + PI * 2;
    }
}
