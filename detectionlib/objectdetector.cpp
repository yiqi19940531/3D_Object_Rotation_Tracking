/**************************************************************************************************
 **************************************************************************************************
 
     BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)
     
     Copyright (c) 2014 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.
     
     
     Redistribution and use in source and binary forms, with or without modification,
     are permitted provided that the following conditions are met:
     
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of the copyright holder nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.
     
     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
     THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
     OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
     OF THE POSSIBILITY OF SUCH DAMAGE.
 
 **************************************************************************************************
 **************************************************************************************************/

#include "objectdetector.h"


ObjectDetector::ObjectDetector(const vector<string> &images, Feat::Code feat, Desc::Code desc, ObjectRotator *rotator = nullptr) :
classifiers(images.size()), filter(4), percent(0), faces(images.size()), rotator(rotator)
{
    
    Space yaw(-30, 30, 60);
    Space pitch(-30,30,60);
    Space roll(-10,10,20);
    Space scale(1*pow(.8,7), 1.0, .8);
    //Space scale(1*pow(.8,3), 1.0, .8);
    
    double fov = 37;
    int numViewPoints = 1000;
    int numKeyPointsPerViewPoint = 70;

    measurements.setTo(Scalar(0));
    kalmanForRotation.initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);

    ViewParams params(fov,
                      yaw,
                      pitch,
                      roll,
                      scale,
                      numViewPoints,
                      numKeyPointsPerViewPoint,
                      feat,
                      desc);
    
    featDetector = Train::create(feat);
    descExtractor= Train::create(desc);
    
    countMissingFrames = MAX_MISS_FRAMES;



    for (int i = 0; i < classifiers.size(); i++)
    {
        stringstream ss;
        ss << images[i];
        ss << ".database";
        string modelFilename = ss.str();
        
        FILE * exists = fopen(modelFilename.c_str(), "r");
        if ( !exists ) {
            Mat referenceImage = imread(images[i], CV_LOAD_IMAGE_GRAYSCALE);

            int m = max(referenceImage.rows, referenceImage.cols);
            if (m > 600) {
                float r = (float) 640.f / m;
                resize(referenceImage, referenceImage,
                       Size(referenceImage.cols * r, referenceImage.rows * r));
            }
            
            vector<vector<StablePoint> > database;
            Train::trainFromReferenceImage(referenceImage, params, database);

            vector<uint16_t> index = Train::computeDescIndices(database, INDEX_SIZE);
            classifiers[i].setIndexPositions(index);
            Train::computeIndices(database, index);
            
            
            classifiers[i].initialize(referenceImage.size(),database, feat, desc );
            classifiers[i].saveModel(modelFilename);
            database.clear();
        }
        else
        {
            fclose(exists);
            classifiers[i].loadModel(modelFilename);
            
        }
    }
    

}



void toGray(const Mat &frame, Mat &gray)
{
    if (frame.channels() > 1)
        cvtColor(frame, gray, CV_BGR2GRAY);
    else
        frame.copyTo(gray);
}

void toRGB(const Mat &frame, Mat &rgb)
{
    if (frame.channels() == 1)
        cvtColor(frame, rgb, CV_GRAY2RGB);
    else
        frame.copyTo(rgb);
}

void ObjectDetector::operator()(const size_t frame_number, const Mat &frameInput, Mat &frameOut)
{



    //camera calibration part
    cameraCalibration calibration;
    int key = waitKey(1);
    if(key == Keys::TAB){
        if(checkBoardFrames.size() < 10 && camera_calibration_stage){
            cout<<"Do not collect enough frame to calibrate the camera!"<<endl;
        }
        if(checkBoardFrames.size() >= 10){
            calibration.calculateCameraCalibration(checkBoardFrames,camera_matrix,dist_coeffs);
            camera_calibration_stage = false;
            checkBoardFrames.clear();
            return;
        }
        if(checkBoardFrames.size() == 0 && !camera_calibration_stage){
            cout<<"Start to collect frame."<<endl;
            camera_calibration_stage = true;
        }
    }

    if(camera_calibration_stage){
        if (key == Keys::c)
        {
            if(calibration.checkFrame(frameInput)){
                checkBoardFrames.push_back(frameInput);
                cout<<"Currently have "<< checkBoardFrames.size()<<" frame."<<endl;
            } else{
                cout<<"Cannot capture enough corner points!"<<endl;
            }

        }
        frameInput.copyTo(frameOut);
        return;
    }

    vector<Point2f> corners;
    Mat frame;
    toRGB(frameInput, frameOut);
    toGray(frameInput, frame);

    Mat mask = Mat::ones(frame.size(), CV_8UC1);

    int levels = 3;
    GaussianBlur(frame, frame, Size(3,3), 3, 3);  //gao si lu bo
    vector<Mat> frames;
    vector<Mat> masks;
    buildPyramid(frame, frames, levels);
//    buildPyramid(mask, masks, levels);
    vector<vector<KeyPoint> > keypoints;
    vector<vector<uint16_t> > indices;
    vector<Mat> descriptors;
//    clock_t B = clock();
//    featDetector->detect(frames, keypoints);
    featDetector->detect(frames, keypoints, masks);
//    clock_t C = clock();
    descExtractor->compute(frames, keypoints, descriptors);
//    clock_t D = clock();

    int idx = 0, maxInliers = 0;
    bool possibleObjectView = false;
    vector<Point2f> oPts, sPts;
    for (int i = 0; i < classifiers.size(); i++)
    {
        vector<DMatch> matches;
        vector<Point2f> objPts;
        vector<Point2f> scnPts;
        
        vector<vector<uint16_t> > idxs;
        Index::getDescPyIndices(descriptors, idxs, classifiers[i].getIndexPositions());
        classifiers[i].match(keypoints, descriptors, idxs, objPts, scnPts, matches);
        
       // classifiers[i].match(keypoints, descriptors, indices, objPts, scnPts, matches);
        //8 Points needed for fundamental matrix computation
        if (matches.size() < 8)
            continue;
        int inliers = FernUtils::numInliers(objPts, scnPts);

        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            idx = i;
            oPts = objPts;
            sPts = scnPts;
            possibleObjectView = true;
        }
    }
//    clock_t E = clock();
    Scalar purple(255,0,255);
    Scalar red(0,0,255);
    Scalar blue(255,0,0);
    Scalar green(0,255,0);
    Point2f shift(0,0);
    vector<Point2f> fc;
    
    if (possibleObjectView)
    {
        Mat F, H;

        Mat pointMask_H;

        vector<Point2f> oFInliners, sFInliers, corners, fCorners, oHInliners, sHInliers;
        vector<Vec3f> eLines;
        const vector<Point2f> &clCorners = classifiers[idx].getCorners();
        FernUtils::computeFundamentalMatrix(oPts, sPts, clCorners, F, oFInliners, sFInliers, eLines);

        int realInliers = FernUtils::computeHomography(oFInliners, sFInliers, clCorners, H, corners, oHInliners, sHInliers, pointMask_H);

        bool canCalculateF = false;
        if(oHInliners.size() > 8){
            canCalculateF = true;
        }

        if (realInliers > 4)
        {
            if (!faces[idx])
            {
                faces[idx] = true;
                cout << faces.size() << " " << 100/faces.size() << endl;
                percent += 100.0 / faces.size();
            }

            FernUtils::refineCornersWithEpipolarLines(eLines, corners, fCorners);
            corners = filter.correct(fCorners);
            lastKnowCorners  = corners;
            FernUtils::drawObject(frameOut, corners, purple, blue, shift, percent);
             FernUtils::drawCorners(frameOut, corners, green, shift);
            rect = boundingRect(corners);
            stringstream ss;
            ss << "Object View Detected: " << (idx + 1);
            if (rotator && canCalculateF)
            {

                std::vector<cv::Point3d> model_points;
                model_points.emplace_back(clCorners[0].x, clCorners[0].y, 0.0f); //3
                model_points.emplace_back(clCorners[1].x, clCorners[1].y, 0.0f); //0
                model_points.emplace_back(clCorners[2].x, clCorners[2].y, 0.0f); //1
                model_points.emplace_back(clCorners[3].x, clCorners[3].y, 0.0f); //2

                std::vector<cv::Point2d> image_points;
                image_points.emplace_back(corners[0].x, corners[0].y);
                image_points.emplace_back(corners[1].x, corners[1].y);
                image_points.emplace_back(corners[2].x, corners[2].y);
                image_points.emplace_back(corners[3].x, corners[3].y);

                //add all points for solve PnP method
                for(int i = 0; i < oHInliners.size(); i++){
                    model_points.emplace_back(oHInliners[i].x, oHInliners[i].y, 0.0f);
                    image_points.emplace_back(sHInliers[i].x, sHInliers[i].y);
                }

                cv::Mat rotationMatrix;
                extractRotMat extractRotationMat;
                bool if_have_valid_solution = false;
                cv::Mat rotation_vector;
                cv::Mat translation_vector;
                cv::Mat rotation_matrix;
                int methods = 3; // 0 for F->E->R, 1 for H->R, 2 for find E->R

                if(methods == 0){  //recover Pose from Essential Matrix
                    Mat F_accuracy, E, camera_matrix2, translation_matrix_E, Tra;
                    F_accuracy = cv::findFundamentalMat(oFInliners,sFInliers);
                    camera_matrix2 = (cv::Mat_<double >(3,3) << 640, 0, 320,0, 640, 320,0,0,1);
                    E = (camera_matrix2.t())*F_accuracy*camera_matrix;
                    extractRotationMat.recoverPoseFromEssenialMat(E,camera_matrix,oFInliners,sFInliers,rotationMatrix,translation_matrix_E,Tra);
                    if_have_valid_solution = true;
                } else if(methods == 1){ //find rotation matrix from homography matrix
                    vector<Mat> rotation_vectors, translate_vectors, normals;
                    Mat camera_matrix2 = (cv::Mat_<double >(3,3) << 640, 0, 320,0, 640, 240,0,0,1);
                    extractRotationMat.decomposeHomographyMat(H,camera_matrix2,rotation_vectors,translate_vectors,normals);
                    if_have_valid_solution = extractRotationMat.filterHomographyDecompByVisibleRefpoints(rotation_vectors,normals,oFInliners,sFInliers,rotationMatrix,pointMask_H);
                    //if_have_valid_solution = filterHomographyDecompByVisibleRefpoints2(rotation_vectors,normals,oFInliners,sFInliers,pointMask_H,rotation_from_H);
                } else if(methods == 2){  //findEssentialMat
                    Mat E_direct, rotation_matrix_direct_E,translation_matrix_direct_E,Tra, mask_E, camera_matrix2;
                    Mat translation_temp_estimated(3,1,CV_64FC1);
                    camera_matrix2 = (cv::Mat_<double >(3,3) << 640, 0, 320,0, 640, 240,0,0,1);
                    E_direct = extractRotationMat.findEssentialMatrix(oFInliners,sFInliers,camera_matrix2,0.999,1.0,mask_E);
                    extractRotationMat.recoverPoseFromEssenialMat(E_direct,camera_matrix2,oFInliners,sFInliers,rotationMatrix,translation_matrix_direct_E,Tra);
//                    kalmanForRotation.fillMeasurements(measurements,translation_matrix_direct_E,rotation_matrix_direct_E);
//                    kalmanForRotation.updateKalmanFilter(KF,measurements,translation_temp_estimated,rotationMatrix);
                    if_have_valid_solution = true;
                } else if(methods == 3){  //SolvePNP method
                    if_have_valid_solution = true;
                    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
                    normalize(rotation_vector, rotation_vector, 1, 0, NORM_L1);
                    cv::Rodrigues(rotation_vector,rotationMatrix);
                }

//                double focal_length = frame.cols; // Approximate focal length.
//                Point2d center = cv::Point2d(frame.cols/2.0f, frame.rows/2.0f);
//                cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
//                cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion





//                vector<Point3d> endpoints_3D;
//                vector<Point2d> endpoints_2D;
//                endpoints_3D.emplace_back(0, 0, -1.0);
//                projectPoints(endpoints_3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, endpoints_2D);
//                cv::line(frameOut, image_points[0], endpoints_2D[0], purple, 2);


                if(if_have_valid_solution) {

                    cv::Vec3f eulerXYZ = extractRotationMat.eulerAngle(rotationMatrix);

                    // Use the face index and the face normal to rotate.
                    //const float increment = 1.0f / faces.size();
                    const float increment = 1.0f / 4.0f;
                    float faceIndexRotation = idx * increment * 2.0f * ObjectRotator::PI;
                    rotator->setRotation_x(eulerXYZ[0], faceIndexRotation);
                    rotator->setRotation_y(-eulerXYZ[1], faceIndexRotation);
                    rotator->setRotation_z(eulerXYZ[2]);

                    //methods for three-degree freedom rotation(Not completely completed)
//                    switch (idx){
//                        case 0:
//                            if( -0.3 < eulerXYZ[2] && eulerXYZ[2] < 0.3 ){
//                                rotator->setRotation_x(eulerXYZ[0],0);
//                                rotator->setRotation_y(eulerXYZ[1], 0);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                            } else{
//                                rotator->setRotation_x(eulerXYZ[0],0);
//                                rotator->setRotation_y(-eulerXYZ[1], 0);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                            }
////                            if(  -1.87 < eulerXYZ[2] && eulerXYZ[2] < -1.27){
////                                rotator->setRotation_y(-eulerXYZ[1], 0);
////                                rotator->setRotation_z(eulerXYZ[2]);
////                            }
//                            break;
//                        case 1:
//                            if( -0.3 < eulerXYZ[2] && eulerXYZ[2] < 0.3 ){
//                                rotator->setRotation_y(eulerXYZ[1], 1.57);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"1"<<endl;
//                            } else{
//                                rotator->setRotation_y(-eulerXYZ[1], -4.71);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"2"<<endl;
//                            }
//                            break;
//                        case 2:
//                            if( -0.3 < eulerXYZ[2] && eulerXYZ[2] < 0.3 ){
//                                rotator->setRotation_y(eulerXYZ[1], 3.14);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"1"<<endl;
//                            } else{
//                                rotator->setRotation_y(-eulerXYZ[1], -3.14);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"2"<<endl;
//                            }
//                            break;
//                        case 3:
//                            if( -0.3 < eulerXYZ[2] && eulerXYZ[2] < 0.3 ){
//                                rotator->setRotation_y(eulerXYZ[1], 4.71);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"1"<<endl;
//                            } else{
//                                rotator->setRotation_y(-eulerXYZ[1], -1.57);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                                cout<<"2"<<endl;
//                            }
//                            break;
//                        case 4:
//                            if( -0.3 < eulerXYZ[2] && eulerXYZ[2] < 0.3 ){
//                                rotator->setRotation_x(eulerXYZ[0],1.57);
//                                rotator->setRotation_y(eulerXYZ[1], 0);
//                                rotator->setRotation_z(eulerXYZ[2]);
//                            }
//                            break;
//
//                    }

                }
            }

            putText(frameOut, ss.str() , Point(40,30), FONT_HERSHEY_DUPLEX, 1, blue);
            rect = boundingRect(corners);
            countMissingFrames = 0;
        }
        else
        {
            countMissingFrames++;
            if (countMissingFrames < MAX_MISS_FRAMES)
            {
                corners = filter.predict();
                rect = boundingRect(corners);
                //rectangle(frameOut, rect.tl(), rect.br(), blue, 4);
                FernUtils::drawObject(frameOut, corners, purple, blue, shift, percent);
                FernUtils::drawCorners(frameOut, corners, green, shift);
            }
            else
                rect = Rect(0,0,frame.cols, frame.rows);
        }

    }
    if (lastKnowCorners.size() == 4)
        filter.correct(lastKnowCorners);
    
//    clock_t FE = clock();

//    printf("Pre: %.2f (%.2f) Detect: %.2f (%.2f) Extract: %.2f (%.2f) Match: %.2f (%.2f) R+K: %.2f (%.2f)\n",
//           double(B-A)/CLOCKS_PER_SEC,
//           1.0/(double(B-A)/CLOCKS_PER_SEC),
//           double(C-B)/CLOCKS_PER_SEC,
//           1.0/(double(C-B)/CLOCKS_PER_SEC),
//           double(D-C)/CLOCKS_PER_SEC,
//           1.0/(double(D-C)/CLOCKS_PER_SEC),
//           double(E-D)/CLOCKS_PER_SEC,
//           1.0/(double(E-D)/CLOCKS_PER_SEC),
//           double(FE-E)/CLOCKS_PER_SEC,
//           1.0/(double(FE-E)/CLOCKS_PER_SEC));

    if (rotator)
    {
        rotator->renderCube();
    }

    frames.clear();
    masks.clear();
    keypoints.clear();
    indices.clear();
    descriptors.clear();
}


