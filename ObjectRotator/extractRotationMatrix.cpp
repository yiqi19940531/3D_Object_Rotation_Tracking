//
// Created by yiqi on 19-10-24.
//

#include "extractRotationMatrix.h"

int extractRotMat::recoverPoseFromEssenialMat(const _InputArray &essentialMat, const _InputArray &K, const _InputArray &p1,
                                              const _InputArray &p2, const _OutputArray &oR, const _OutputArray &ot,
                                              const _OutputArray &triangulatedPoints) {
    //cv::CV_INSTRUMENT_REGION();
    Mat points1, points2, cameraMatrix;
    p1.getMat().convertTo(points1, CV_64F);
    p2.getMat().convertTo(points2, CV_64F);
    K.getMat().convertTo(cameraMatrix, CV_64F);

    int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
               points1.type() == points2.type());

    CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

    if (points1.channels() > 1)
    {
        points1 = points1.reshape(1, npoints);
        points2 = points2.reshape(1, npoints);
    }

    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    points1.col(0) = (points1.col(0) - cx) / fx;
    points2.col(0) = (points2.col(0) - cx) / fx;
    points1.col(1) = (points1.col(1) - cy) / fy;
    points2.col(1) = (points2.col(1) - cy) / fy;

    points1 = points1.t();
    points2 = points2.t();

    Mat R1, R2, t;

    essentialMatSVD(essentialMat,R1,R2,t);
    Mat P0 = Mat::eye(3, 4, R1.type());
    Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
    P1(Range::all(), Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
    P2(Range::all(), Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
    P3(Range::all(), Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
    P4(Range::all(), Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

    std::vector<Mat> allTriangulations(4);
    Mat Q;

    triangulatePoints(P0, P1, points1, points2, Q);
    if(triangulatedPoints.needed())
        Q.copyTo(allTriangulations[0]);

    double distanceThresh = 50;
    Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask1 = (Q.row(2) < distanceThresh) & mask1;
    Q = P1 * Q;
    mask1 = (Q.row(2) > 0) & mask1;
    mask1 = (Q.row(2) < distanceThresh) & mask1;

    triangulatePoints(P0, P2, points1, points2, Q);
    if(triangulatedPoints.needed())
        Q.copyTo(allTriangulations[1]);
    Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask2 = (Q.row(2) < distanceThresh) & mask2;
    Q = P2 * Q;
    mask2 = (Q.row(2) > 0) & mask2;
    mask2 = (Q.row(2) < distanceThresh) & mask2;

    triangulatePoints(P0, P3, points1, points2, Q);
    if(triangulatedPoints.needed())
        Q.copyTo(allTriangulations[2]);
    Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask3 = (Q.row(2) < distanceThresh) & mask3;
    Q = P3 * Q;
    mask3 = (Q.row(2) > 0) & mask3;
    mask3 = (Q.row(2) < distanceThresh) & mask3;

    triangulatePoints(P0, P4, points1, points2, Q);
    if(triangulatedPoints.needed())
        Q.copyTo(allTriangulations[3]);
    Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask4 = (Q.row(2) < distanceThresh) & mask4;
    Q = P4 * Q;
    mask4 = (Q.row(2) > 0) & mask4;
    mask4 = (Q.row(2) < distanceThresh) & mask4;

    mask1 = mask1.t();
    mask2 = mask2.t();
    mask3 = mask3.t();
    mask4 = mask4.t();

    // If _mask is given, then use it to filter outliers.
//    if (!_mask.empty())
//    {
//        Mat mask = _mask.getMat();
//        CV_Assert(mask.size() == mask1.size());
//        bitwise_and(mask, mask1, mask1);
//        bitwise_and(mask, mask2, mask2);
//        bitwise_and(mask, mask3, mask3);
//        bitwise_and(mask, mask4, mask4);
//    }
//    if (_mask.empty() && _mask.needed())
//    {
//        _mask.create(mask1.size(), CV_8U);
//    }

    CV_Assert(oR.needed() && ot.needed());
    oR.create(3, 3, R1.type());
    ot.create(3, 1, t.type());

    int good1 = countNonZero(mask1);
    int good2 = countNonZero(mask2);
    int good3 = countNonZero(mask3);
    int good4 = countNonZero(mask4);

    //int good1 = 5, good2 = 2, good3 = 3, good4 = 4;

    if (good1 >= good2 && good1 >= good3 && good1 >= good4)
    {
        if(triangulatedPoints.needed()) allTriangulations[0].copyTo(triangulatedPoints);
        R1.copyTo(oR);
        t.copyTo(ot);
//        cout<<"R1";
//        cout<<endl;
        //if (_mask.needed()) mask1.copyTo(_mask);
        return good1;
    }
    else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
    {
        if(triangulatedPoints.needed()) allTriangulations[1].copyTo(triangulatedPoints);
        R2.copyTo(oR);
        t.copyTo(ot);
//        cout<<"R2";
//        cout<<endl;
        //if (_mask.needed()) mask2.copyTo(_mask);
        return good2;
    }
    else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
    {
        if(triangulatedPoints.needed()) allTriangulations[2].copyTo(triangulatedPoints);
        t = -t;
        R1.copyTo(oR);
        t.copyTo(ot);
//        cout<<"R3";
//        cout<<endl;
        //if (_mask.needed()) mask3.copyTo(_mask);
        return good3;
    }
    else
    {
        if(triangulatedPoints.needed()) allTriangulations[3].copyTo(triangulatedPoints);
        t = -t;
        R2.copyTo(oR);
        t.copyTo(ot);
//        cout<<"R4";
//        cout<<endl;
        //if (_mask.needed()) mask4.copyTo(_mask);
        return good4;
    }
}

void extractRotMat::essentialMatSVD(const _InputArray &_E, const _OutputArray &_R1, const _OutputArray &_R2,
                                    const _OutputArray &_t) {
    Mat E = _E.getMat().reshape(1, 3);
    CV_Assert(E.cols == 3 && E.rows == 3);
    Mat D, U, Vt;
    cv::SVDecomp(E,D,U,Vt);
//    if (determinant(U) < 0) U *= -1.;
//    if (determinant(Vt) < 0) Vt *= -1.;
    Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    W.convertTo(W, E.type());

    Mat R1, R2, t;
    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;
    t = U.col(2) * 1.0;

    if(determinant(R1) < 0 ){
        R1 = - R1;
    }

    if(determinant(R2) < 0 ){
        R2 = - R2;
    }
//    cout<<R1;
//    cout<<endl;
//    cout<<R2;
//    cout<<endl;
    R1.copyTo(_R1);
    R2.copyTo(_R2);
    t.copyTo(_t);
}

int extractRotMat::decomposeHomographyMat(const _InputArray &_H, const _InputArray &_K, const _OutputArray &_rotations,
                                          const _OutputArray &_translations, const _OutputArray &_normals) {
    Mat H = _H.getMat().reshape(1, 3);
    CV_Assert(H.cols == 3 && H.rows == 3);
    //cout<<H<<endl;

    Mat K = _K.getMat().reshape(1, 3);
    CV_Assert(K.cols == 3 && K.rows == 3);
    //cout<<K<<endl;

    vector<CameraMotion> motions;

    decomposeHomography(H, K, motions);

    int nsols = static_cast<int>(motions.size());
    int depth = CV_64F; //double precision matrices used in CameraMotion struct


    _rotations.create(nsols, 1, depth);
    for (int k = 0; k < nsols; ++k ) {
        _rotations.getMatRef(k) = Mat(motions[k].R);
    }


    _translations.create(nsols, 1, depth);
    for (int k = 0; k < nsols; ++k ) {
        _translations.getMatRef(k) = Mat(motions[k].t);
    }



    _normals.create(nsols, 1, depth);
    for (int k = 0; k < nsols; ++k ) {
        _normals.getMatRef(k) = Mat(motions[k].n);
    }

    return nsols;
}

Matx33d extractRotMat::normalize(const Matx33d &H, const Matx33d &K) {
    return K.inv() * H * K;
}

void extractRotMat::removeScale() {
    Mat W;
    SVD::compute(_Hnorm, W);
    _Hnorm = _Hnorm * (1.0/W.at<double>(1));
}

void extractRotMat::decompose(std::vector<CameraMotion>& camMotions){
    const double epsilon = 0.001;
    Matx33d S;

    //S = H'H - I
    S = _Hnorm.t() * _Hnorm;
    S(0, 0) -= 1.0;
    S(1, 1) -= 1.0;
    S(2, 2) -= 1.0;

    //check if H is rotation matrix
    if( norm(S, NORM_INF) < epsilon) {
        CameraMotion motion;
        motion.R = Matx33d(_Hnorm);
        motion.t = Vec3d(0, 0, 0);
        motion.n = Vec3d(0, 0, 0);
        camMotions.push_back(motion);
        return;
    }

    //! Compute nvectors
    Vec3d npa, npb;

    double M00 = oppositeOfMinor(S, 0, 0);
    double M11 = oppositeOfMinor(S, 1, 1);
    double M22 = oppositeOfMinor(S, 2, 2);

    double rtM00 = sqrt(M00);
    double rtM11 = sqrt(M11);
    double rtM22 = sqrt(M22);

    double M01 = oppositeOfMinor(S, 0, 1);
    double M12 = oppositeOfMinor(S, 1, 2);
    double M02 = oppositeOfMinor(S, 0, 2);

    int e12 = signd(M12);
    int e02 = signd(M02);
    int e01 = signd(M01);

    double nS00 = abs(S(0, 0));
    double nS11 = abs(S(1, 1));
    double nS22 = abs(S(2, 2));

    //find max( |Sii| ), i=0, 1, 2
    int indx = 0;
    if(nS00 < nS11){
        indx = 1;
        if( nS11 < nS22 )
            indx = 2;
    }
    else {
        if(nS00 < nS22 )
            indx = 2;
    }

    switch (indx) {
        case 0:
            npa[0] = S(0, 0),               npb[0] = S(0, 0);
            npa[1] = S(0, 1) + rtM22,       npb[1] = S(0, 1) - rtM22;
            npa[2] = S(0, 2) + e12 * rtM11, npb[2] = S(0, 2) - e12 * rtM11;
            break;
        case 1:
            npa[0] = S(0, 1) + rtM22,       npb[0] = S(0, 1) - rtM22;
            npa[1] = S(1, 1),               npb[1] = S(1, 1);
            npa[2] = S(1, 2) - e02 * rtM00, npb[2] = S(1, 2) + e02 * rtM00;
            break;
        case 2:
            npa[0] = S(0, 2) + e01 * rtM11, npb[0] = S(0, 2) - e01 * rtM11;
            npa[1] = S(1, 2) + rtM00,       npb[1] = S(1, 2) - rtM00;
            npa[2] = S(2, 2),               npb[2] = S(2, 2);
            break;
        default:
            break;
    }

    double traceS = S(0, 0) + S(1, 1) + S(2, 2);
    double v = 2.0 * sqrt(1 + traceS - M00 - M11 - M22);

    double ESii = signd(S(indx, indx)) ;
    double r_2 = 2 + traceS + v;
    double nt_2 = 2 + traceS - v;

    double r = sqrt(r_2);
    double n_t = sqrt(nt_2);

    Vec3d na = npa / norm(npa);
    Vec3d nb = npb / norm(npb);

    double half_nt = 0.5 * n_t;
    double esii_t_r = ESii * r;

    Vec3d ta_star = half_nt * (esii_t_r * nb - n_t * na);
    Vec3d tb_star = half_nt * (esii_t_r * na - n_t * nb);

    camMotions.resize(4);

    Matx33d Ra, Rb;
    Vec3d ta, tb;

    //Ra, ta, na
    findRmatFrom_tstar_n(ta_star, na, v, Ra);
    ta = Ra * ta_star;

    camMotions[0].R = Ra;
    camMotions[0].t = ta;
    camMotions[0].n = na;

    //Ra, -ta, -na
    camMotions[1].R = Ra;
    camMotions[1].t = -ta;
    camMotions[1].n = -na;

    //Rb, tb, nb
    findRmatFrom_tstar_n(tb_star, nb, v, Rb);
    tb = Rb * tb_star;

    camMotions[2].R = Rb;
    camMotions[2].t = tb;
    camMotions[2].n = nb;

    //Rb, -tb, -nb
    camMotions[3].R = Rb;
    camMotions[3].t = -tb;
    camMotions[3].n = -nb;
}


void extractRotMat::decomposeHomography(const Matx33d &H, const cv::Matx33d &K,
                                        std::vector<CameraMotion> &camMotions) {
    //normalize homography matrix with intrinsic camera matrix
    _Hnorm = normalize(H,K);
    //remove scale of the normalized homography
    removeScale();
    //apply decomposition
    decompose(camMotions);
}

bool extractRotMat::filterHomographyDecompByVisibleRefpoints(const _InputArray &_rotations, const _InputArray &_normals,
                                                             const _InputArray &_beforeRectifiedPoints,
                                                             const _InputArray &_afterRectifiedPoints,
                                                             const _OutputArray &_possibleSolutions,
                                                             const _InputArray &_pointsMask) {
    CV_Assert(_beforeRectifiedPoints.type() == CV_32FC2 && _afterRectifiedPoints.type() == CV_32FC2);
    CV_Assert(_pointsMask.empty() || _pointsMask.type() == CV_8U);
    bool flag = false;

    Mat beforeRectifiedPoints = _beforeRectifiedPoints.getMat();
    Mat afterRectifiedPoints = _afterRectifiedPoints.getMat();
    Mat pointsMask = _pointsMask.getMat();
    int nsolutions = (int)_rotations.total();
    int npoints = (int)beforeRectifiedPoints.total();
    CV_Assert(pointsMask.empty() || pointsMask.checkVector(1, CV_8U) == npoints);
    const uchar* pointsMaskPtr = pointsMask.data;

    std::vector<uchar> solutionMask(nsolutions, (uchar)1);
    std::vector<Mat> normals(nsolutions);
    std::vector<Mat> rotnorm(nsolutions);
    Mat R;

    for( int i = 0; i < nsolutions; i++ )
    {
        _normals.getMat(i).convertTo(normals[i], CV_64F);
        CV_Assert(normals[i].total() == 3);
        _rotations.getMat(i).convertTo(R, CV_64F);
        rotnorm[i] = R*normals[i];
        CV_Assert(rotnorm[i].total() == 3);
    }

    for( int j = 0; j < npoints; j++ )
    {
        if( !pointsMaskPtr || pointsMaskPtr[j] )
        {
            Point2f prevPoint = beforeRectifiedPoints.at<Point2f>(j);
            Point2f currPoint = afterRectifiedPoints.at<Point2f>(j);

            for( int i = 0; i < nsolutions; i++ )
            {
                if( !solutionMask[i] )
                    continue;

                const double* normal_i = normals[i].ptr<double>();
                const double* rotnorm_i = rotnorm[i].ptr<double>();
                double prevNormDot = normal_i[0]*prevPoint.x + normal_i[1]*prevPoint.y + normal_i[2];
                double currNormDot = rotnorm_i[0]*currPoint.x + rotnorm_i[1]*currPoint.y + rotnorm_i[2];

                if (prevNormDot <= 0 || currNormDot <= 0)
                    solutionMask[i] = (uchar)0;
            }
        }
    }

    std::vector<int> possibleSolutions;
    for( int i = 0; i < nsolutions; i++ )
        if( solutionMask[i] ){
            //possibleSolutions.push_back(i);
            flag = true;
            _rotations.getMat(i).copyTo(_possibleSolutions);
        }

    return flag;
}

bool extractRotMat::filterHomographyDecompByVisibleRefpoints2(const _InputArray &rotations, const _InputArray &normals,
                                                              const _InputArray &_beforeRectifiedPoints,
                                                              const _InputArray &_afterRectifiedPoints,
                                                              const _InputArray &_pointsMask,
                                                              const _OutputArray &_possibleSolutions) {
    CV_Assert(_beforeRectifiedPoints.type() == CV_32FC2 && _afterRectifiedPoints.type() == CV_32FC2 && (_pointsMask.empty() || _pointsMask.type() == CV_8U));

    bool flag = false;

    Mat beforeRectifiedPoints = _beforeRectifiedPoints.getMat(), afterRectifiedPoints = _afterRectifiedPoints.getMat(), pointsMask = _pointsMask.getMat();

    Mat possibleSolutions;

    for (int solutionIdx = 0; solutionIdx < rotations.size().area(); solutionIdx++)
    {
        bool solutionValid = true;

        for (int pointIdx = 0; pointIdx < beforeRectifiedPoints.size().area(); pointIdx++) {
            if (pointsMask.empty() || pointsMask.at<bool>(pointIdx))
            {
                Mat tempAddMat = Mat(1, 1, CV_64F, double(1));

                Mat tempPrevPointMat = Mat(beforeRectifiedPoints.at<Point2f>(pointIdx));
                tempPrevPointMat.convertTo(tempPrevPointMat, CV_64F);
                tempPrevPointMat.push_back(tempAddMat);

                Mat tempCurrPointMat = Mat(afterRectifiedPoints.at<Point2f>(pointIdx));
                tempCurrPointMat.convertTo(tempCurrPointMat, CV_64F);
                tempCurrPointMat.push_back(tempAddMat);

                double prevNormDot = tempPrevPointMat.dot(normals.getMat(solutionIdx));
                double currNormDot = tempCurrPointMat.dot(rotations.getMat(solutionIdx) * normals.getMat(solutionIdx));

                if (prevNormDot <= 0 || currNormDot <= 0)
                {
                    solutionValid = false;
                    break;
                }
            }
        }
        if (solutionValid)
        {
            //possibleSolutions.push_back(solutionIdx);
            rotations.getMat(solutionIdx).copyTo(_possibleSolutions);
            flag = true;
        }
    }

    return flag;
}

cv::Mat extractRotMat::findEssentialMatrix(const _InputArray &_points1, const _InputArray &_points2,
                                           const _InputArray &_cameraMatrix, double prob, double threshold,
                                           const _OutputArray &_mask) {
    Mat points1, points2, cameraMatrix;
    _points1.getMat().convertTo(points1, CV_64F);
    _points2.getMat().convertTo(points2, CV_64F);
    _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

    int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
               points1.type() == points2.type());

    CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

    if (points1.channels() > 1)
    {
        points1 = points1.reshape(1, npoints);
        points2 = points2.reshape(1, npoints);
    }

    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    points1.col(0) = (points1.col(0) - cx) / fx;
    points2.col(0) = (points2.col(0) - cx) / fx;
    points1.col(1) = (points1.col(1) - cy) / fy;
    points2.col(1) = (points2.col(1) - cy) / fy;

    // Reshape data to fit opencv ransac function
    points1 = points1.reshape(2, npoints);
    points2 = points2.reshape(2, npoints);

    threshold /= (fx+fy)/2;

    Mat E;

    createRANSACPointSetRegistrator(makePtr<EMEstimatorCallback>(), 5, threshold, prob)->run(points1, points2, E, _mask); //try with 1000 and without

    return E;
}

cv::Vec3f extractRotMat::eulerAngle(Mat &R) {
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}