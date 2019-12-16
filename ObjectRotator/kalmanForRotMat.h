//
// Created by yiqi on 19-10-27.
//

#ifndef DETECTION3D_KALMANFORROTMAT_H
#define DETECTION3D_KALMANFORROTMAT_H

#include <opencv2/video/tracking.hpp>


using namespace cv;

class kalmanFilterForRotationMat{
public:

    /* Initialize the Kalman filter
     * &KF: The object of Kalman filter
     * nStates: the number of states
     * nMeasurements: the number of measured states (three for rotation and three for translation)
     * nInputs: the number of control actions
     * dt: time between measurements (1/FPS)
     */
    void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);

    /* Used to find the final estimated value for translation and rotation.
     * &KF: The object of Kalman filter
     * nMeasurements: the number of measured states (three for rotation and three for translation)
     * translation_estimated: estimated translation value
     * rotation_estimated: estimated rotation value
     */
    void updateKalmanFilter( KalmanFilter &KF, Mat &measurement,
                             Mat &translation_estimated, Mat &rotation_estimated);


    /* used to update the filter
     * nMeasurements: the number of measured states (three for rotation and three for translation)
     * translation_measured: the measured translation value
     * rotation_measured: the measured rotation value
     */
    void fillMeasurements( Mat &measurements,
                           const Mat &translation_measured, const Mat &rotation_measured);


private:

    // Converts a given Rotation Matrix to Euler angles
    // Convention used is Y-Z-X Tait-Bryan angles
    // Reference code implementation:
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
    cv::Mat rot2euler(const cv::Mat & rotationMatrix);


    // Converts a given Rotation Matrix to Euler angles
    // Convention used is Y-Z-X Tait-Bryan angles
    // Reference code implementation:
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
    cv::Mat euler2rot(const cv::Mat & euler);

};

#endif //DETECTION3D_KALMANFORROTMAT_H
