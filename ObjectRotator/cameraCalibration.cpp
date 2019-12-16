//
// Created by yiqi on 19-10-24.
//

#include "cameraCalibration.h"

void cameraCalibration::calculateCameraCalibration(vector<Mat> &checkBoardFrames, OutputArray K, OutputArray dist_coeffs) {

    cout<<"Start extracting corners...\n";
    int image_count=0;
    Size image_size;
    vector<Point2f> image_points_buf;
    vector<vector<Point2f>> image_points_seq;
    int count= -1 ;

    int countNum = checkBoardFrames.size();
    while(image_count < countNum){
        image_count++;

        cout<<"image_count = "<<image_count<<endl;

        Mat imageInput= checkBoardFrames[image_count - 1];

        if (image_count == 1)
        {
            image_size.width = imageInput.cols;
            image_size.height =imageInput.rows;
            cout<<"image_size.width = "<<image_size.width<<endl;
            cout<<"image_size.height = "<<image_size.height<<endl;
        }

        findChessboardCorners(imageInput,board_size,image_points_buf);

        Mat view_gray;
        cvtColor(imageInput,view_gray,CV_RGB2GRAY);

        find4QuadCornerSubpix(view_gray,image_points_buf,Size(11,11));
        image_points_seq.push_back(image_points_buf);

        drawChessboardCorners(view_gray,board_size,image_points_buf,true);
        String num = to_string(image_count);
        String fileName = "calibration_img/img" + num + ".jpg";
        cout<<fileName<<endl;
        imwrite(fileName,view_gray);

    }

    cout<<"Start calibrating camera ...\n";

    vector<vector<Point3f>> object_points;
    /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
    vector<int> point_counts;
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0));
    vector<Mat> tvecsMat;
    vector<Mat> rvecsMat;

    int i,j,t;
    for (t=0;t<image_count;t++)
    {
        vector<Point3f> tempPointSet;
        for (i=0;i<board_size.height;i++)
        {
            for (j=0;j<board_size.width;j++)
            {
                Point3f realPoint;
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }

    for (i=0;i<image_count;i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }

    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout<<"Calibration finish.\n";

    cameraMatrix.copyTo(K);
    distCoeffs.copyTo(dist_coeffs);

    cout<<"Start to evaluating calibration result.\n";
    double total_err = 0.0;
    double err = 0.0;
    vector<Point2f> image_points2;
    vector<double> err_perImage;
    vector<int> position;

    for (i=0;i<image_count;i++)
    {
        vector<Point3f> tempPointSet=object_points[i];
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);
        vector<Point2f> tempImagePoint = image_points_seq[i];
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/=  point_counts[i];
        std::cout<<"The "<<i+1<<" image average error: "<<err<<" pixel"<<endl;
        err_perImage.push_back(err*1000);
        position.push_back(i + 1);

    }

    std::cout<<"The total error: "<<total_err/image_count<<" pixel"<<endl;
    std::cout<<"Evaluation finish."<<endl;

    cout<<cameraMatrix<<endl;
    cout<<distCoeffs<<endl;

}

bool cameraCalibration::checkFrame(const Mat &frame) {

    vector<Point2f> image_points_buf;
    bool flag = findChessboardCorners(frame,board_size,image_points_buf);
    image_points_buf.clear();
    return flag;

}
