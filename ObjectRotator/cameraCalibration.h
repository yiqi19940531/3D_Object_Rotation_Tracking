//
// Created by yiqi on 19-10-24.
//

#ifndef DETECTION3D_CAMERACALIBRATION_H
#define DETECTION3D_CAMERACALIBRATION_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class cameraCalibration{
public:

    /* This function is used to calibrate the camera
     * Parameters:
     * vector<Mat> &checkBoardFrames: The vector of the input frames.
     * K: Output camera matrix value
     * dist_conffs: Output the distortion coefficients.
     */
    void calculateCameraCalibration(vector<Mat> &checkBoardFrames, OutputArray K, OutputArray dist_coeffs);

    // Check whether current frame is valid
    bool checkFrame(const Mat &frame);

private:

    //check board row and col corner point number
    Size board_size = Size(4,6);

    //The real size for each checkerboard (mm)
    Size square_size = Size(25,25);


};

#endif //DETECTION3D_CAMERACALIBRATION_H
