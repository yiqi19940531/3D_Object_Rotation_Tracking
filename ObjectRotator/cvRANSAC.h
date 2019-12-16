//
// Created by yiqi on 19-10-14.
//

#ifndef DETECTION3D_CVRANSAC_H
#define DETECTION3D_CVRANSAC_H

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

namespace cv
{

    int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters );

    class CV_EXPORTS PointSetRegistrator : public Algorithm
    {
    public:
        class CV_EXPORTS Callback
        {
        public:
            virtual ~Callback() {}
            virtual int runKernel(InputArray m1, InputArray m2, OutputArray model) const = 0;
            virtual void computeError(InputArray m1, InputArray m2, InputArray model, OutputArray err) const = 0;
            virtual bool checkSubset(InputArray, InputArray, int) const { return true; }
        };

        virtual void setCallback(const Ptr<PointSetRegistrator::Callback>& cb) = 0;
        virtual bool run(InputArray m1, InputArray m2, OutputArray model, OutputArray mask) const = 0;
    };

    CV_EXPORTS Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
                                                                        int modelPoints, double threshold,
                                                                        double confidence = 0.99, int maxIters = 1000);

    CV_EXPORTS Ptr<PointSetRegistrator> createLMeDSPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
                                                                       int modelPoints, double confidence = 0.99, int maxIters = 1000);





}

#endif //DETECTION3D_CVRANSAC_H
