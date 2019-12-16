//
// Created by yiqi on 19-10-24.
//

#ifndef DETECTION3D_EXTRACTROTATIONMATRIX_H
#define DETECTION3D_EXTRACTROTATIONMATRIX_H

#include <iostream>
#include "EMEstimator.cpp"

using namespace std;
using namespace cv;

//struct to hold solutions of homography decomposition
typedef struct _CameraMotion {
    cv::Matx33d R; //!< rotation matrix
    cv::Vec3d n; //!< normal of the plane the camera is looking at
    cv::Vec3d t; //!< translation vector
} CameraMotion;

class extractRotMat{
public:

    /*find the rotation matrix from Essential Matrix
     * Parameters:
     * E: The input essential matrix.
     * points1: Array of N 2D points from the first image. The point coordinates should be floating-point (single or double precision).
     * points2:Array of the second image points of the same size and format as points1.
     * K: camera matrix.
     * oR: Recovered relative rotation.
     * ot: Recovered relative translation.
     */
    int recoverPoseFromEssenialMat(cv::InputArray essentialMat, cv::InputArray K, cv::InputArray p1,
            cv::InputArray p2, cv::OutputArray oR, cv::OutputArray ot,cv::OutputArray triangulatedPoints);

    /* find the rotations matrix from Homography Matrix
     * Parameters:
     * H: The input homography matrix between two images.
     * K: The input intrinsic camera calibration matrix.
     * rotations: Array of rotation matrices.
     * translations: Array of translation matrices.
     * normals: Array of plane normal matrices.
     */

    int decomposeHomographyMat(InputArray _H,InputArray _K,OutputArrayOfArrays _rotations,
            OutputArrayOfArrays _translations,OutputArrayOfArrays _normals);

    /* Filters homography decompositions based on additional information.
     * Parameters:
     * rotations: Vector of rotation matrices.
     * normals: Vector of plane normal matrices.
     * beforePoints: Vector of (rectified) visible reference points before the homography is applied.
     * afterPoints: Vector of (rectified) visible reference points after the homography is applied.
     * possibleSolutions: Vector of int indices representing the viable solution set after filtering.
     * pointsMask: optional Mat/Vector of 8u type representing the mask for the inliers as given by the findHomography function.
     */
    bool filterHomographyDecompByVisibleRefpoints(InputArrayOfArrays _rotations,
                                                  InputArrayOfArrays _normals,
                                                  InputArray _beforeRectifiedPoints,
                                                  InputArray _afterRectifiedPoints,
                                                  OutputArray _possibleSolutions,
                                                  InputArray _pointsMask);

    /* Filters homography decompositions based on additional information. (based on test result worse than filterHomographyDecompByVisibleRefpoints())
     * Parameters:
     * rotations: Vector of rotation matrices.
     * normals: Vector of plane normal matrices.
     * beforePoints: Vector of (rectified) visible reference points before the homography is applied.
     * afterPoints: Vector of (rectified) visible reference points after the homography is applied.
     * pointsMask: optional Mat/Vector of 8u type representing the mask for the inliers as given by the findHomography function.
     * possibleSolutions: Vector of int indices representing the viable solution set after filtering.
     */

    bool filterHomographyDecompByVisibleRefpoints2(InputArrayOfArrays rotations,
                                                   InputArrayOfArrays normals,
                                                   InputArray _beforeRectifiedPoints,
                                                   InputArray _afterRectifiedPoints,
                                                   InputArray _pointsMask,
                                                   OutputArray _possibleSolutions);


    /* Calculates an essential matrix from the corresponding points in two images.
     * Parameters:
     * points1: Array of N (N >= 5) 2D points from the first image. The point coordinates should be floating-point (single or double precision).
     * point2: Array of the second image points of the same size and format as points1.
     * _cameraMatrix: Camera matrix.
     * prob: Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of confidence (probability) that the estimated matrix is correct.
     * threshold: Parameter used for RANSAC.
     * mask: Output array of N elements, every element of which is set to 0 for outliers and to 1 for the other points.
     */
    Mat findEssentialMatrix( InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                              double prob, double threshold, OutputArray _mask);

    //This method transform Rotation Matrix to Euler Angle as XYZ order.
    cv::Vec3f eulerAngle(Mat &R);


private:
    cv::Matx33d _Hnorm;

    void essentialMatSVD(cv::InputArray _E, cv::OutputArray _R1, cv::OutputArray _R2, cv::OutputArray _t);

    void decomposeHomography(const Matx33d& H, const cv::Matx33d& K,std::vector<CameraMotion>& camMotions);

    Matx33d normalize(const Matx33d& H, const Matx33d& K);

    void removeScale();

    void decompose(std::vector<CameraMotion>& camMotions);

    double oppositeOfMinor(const Matx33d& M, const int row, const int col){
        int x1 = col == 0 ? 1 : 0;
        int x2 = col == 2 ? 1 : 2;
        int y1 = row == 0 ? 1 : 0;
        int y2 = row == 2 ? 1 : 2;

        return (M(y1, x2) * M(y2, x1) - M(y1, x1) * M(y2, x2));
    }

    inline int signd(const double x)
    {
        return ( x >= 0 ? 1 : -1 );
    }

    void findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R){
        Matx31d tstar_m = Matx31d(tstar);
        Matx31d n_m = Matx31d(n);
        Matx33d I(1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0);

        R = _Hnorm * (I - (2/v) * tstar_m * n_m.t() );
        if (cv::determinant(R) < 0)
        {
            R *= -1;
        }
    }


};



#endif //DETECTION3D_EXTRACTROTATIONMATRIX_H
