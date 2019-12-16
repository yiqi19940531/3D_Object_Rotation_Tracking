//
// Created by dlalancette on 19/02/19.
//

#ifndef DETECTION3D_OBJECTROTATOR_H
#define DETECTION3D_OBJECTROTATOR_H

#include <ml.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>


typedef void (*RenderFunction)(glm::mat4 &);

class ObjectRotator
{
public:
    static constexpr float PI = 3.1416f;

    explicit ObjectRotator(RenderFunction function)
        : render(function)
        , interpolationParam_y(0.0f)
        , desiredAngle_y(0.0f)
        , currentAngle_y(0.0f)
        , interpolationParam_z(0.0f)
        , desiredAngle_z(0.0f)
        , currentAngle_z(0.0f)
    {

    }

    void setRotation(float radians);
    void setRotation(const glm::vec3 &normal, float radians);
    void setRotation_x(const float xAxisRadians, float face_radians);
    void setRotation_y(const float yAxisRadians, float face_radians);
    void setRotation_z(const float zAxisRadians);

    void renderCube();

private:
    RenderFunction render;

    float interpolationParam_x;
    float interpolationParam_y;
    float interpolationParam_z;

    float currentAngle_x;
    float currentAngle_y;
    float currentAngle_z;

    float desiredAngle_x;
    float desiredAngle_y;
    float desiredAngle_z;

};

#endif //DETECTION3D_OBJECTROTATOR_H
