/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Output
#include <iostream>

// Vector
#include <vector>

// >>>>> BFL includes
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <bfl/pdf/linearanalyticconditionalgaussian.h>
// <<<<< BFL includes

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// <<<<< Color to be tracked

// >>>>> State defines
#define _X      1
#define _Y      2
#define _Xdot   3
#define _Ydot   4
#define _W      5
#define _H      6
// <<<<< State defines

// >>>>> Measures defines
#define M_X      1
#define M_Y      2
#define M_W      3
#define M_H      4
// <<<<< Measures defines

// >>>>> System Gaussian
#define MU_SYSTEM_NOISE_X       3.0
#define MU_SYSTEM_NOISE_Y       3.0
#define MU_SYSTEM_NOISE_Xdot    1.0
#define MU_SYSTEM_NOISE_Ydot    1.0
#define MU_SYSTEM_NOISE_W       5.0
#define MU_SYSTEM_NOISE_H       5.0

#define SIGMA_SYSTEM_NOISE_X       1.0
#define SIGMA_SYSTEM_NOISE_Y       1.0
#define SIGMA_SYSTEM_NOISE_Xdot    2.0
#define SIGMA_SYSTEM_NOISE_Ydot    2.0
#define SIGMA_SYSTEM_NOISE_W       3.0
#define SIGMA_SYSTEM_NOISE_H       3.0
// <<<<< System Gaussian

// >>>>> Measure Gaussian
#define MU_MEAS_NOISE_X       3.0
#define MU_MEAS_NOISE_Y       3.0
#define MU_MEAS_NOISE_W       5.0
#define MU_MEAS_NOISE_H       5.0

#define SIGMA_MEAS_NOISE_X       1.0
#define SIGMA_MEAS_NOISE_Y       1.0
#define SIGMA_MEAS_NOISE_W       3.0
#define SIGMA_MEAS_NOISE_H       3.0
// <<<<< Measure Gaussian

int main()
{
    // Camera frame
    cv::Mat frame;

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    // >>>>> System Matrix
    Matrix A(stateSize,stateSize);
    A = 0.0;
    for( int i=1; i<=stateSize; i++ )
    {
        A(i,i)=1.0;
    }
    // <<<<< System Matrix

    // >>>>> System Noise
    ColumnVector sysNoise_Mu(stateSize);
    sysNoise_Mu(_X)     = MU_SYSTEM_NOISE_X; // X
    sysNoise_Mu(_Y)     = MU_SYSTEM_NOISE_Y; // Y
    sysNoise_Mu(_Xdot)  = MU_SYSTEM_NOISE_Xdot; // V_X
    sysNoise_Mu(_Ydot)  = MU_SYSTEM_NOISE_Ydot; // V_Y
    sysNoise_Mu(_W)     = MU_SYSTEM_NOISE_W; // W
    sysNoise_Mu(_H)     = MU_SYSTEM_NOISE_H; // H

    SymmetricMatrix sysNoise_Cov(stateSize);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(_X,_X)       = SIGMA_SYSTEM_NOISE_X;
    sysNoise_Cov(_Y,_Y)       = SIGMA_SYSTEM_NOISE_Y;
    sysNoise_Cov(_Xdot,_Xdot) = SIGMA_SYSTEM_NOISE_Xdot;
    sysNoise_Cov(_Ydot,_Ydot) = SIGMA_SYSTEM_NOISE_Ydot;
    sysNoise_Cov(_W,_W)       = SIGMA_SYSTEM_NOISE_W;
    sysNoise_Cov(_H,_H)       = SIGMA_SYSTEM_NOISE_H;
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    // <<<<< System Noise

    LinearAnalyticConditionalGaussian sys_pdf(A, system_Uncertainty);
    LinearAnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);

    // >>>>> Measures Matrix
    Matrix H(measSize,stateSize);
    H=0.0;

    // Speed is not measured, so the related lines are missing
    /*[1 0 0 0 0 0 0]
    [0 1 0 0 0 0 0]
    [0 0 0 0 0 1 0]
    [0 0 0 0 0 0 1]*/

    int missing = 2;

    H(_X,_X) = 1.0;
    H(_Y,_Y) = 1.0;
    H(_W-missing, _W) = 1.0;
    H(_H-missing, _H) = 1.0;
    // <<<<< Measures Matrix

    // >>>>> Measures Noise
    ColumnVector measNoise_Mu(measSize);
    measNoise_Mu(M_X) = MU_MEAS_NOISE_X;
    measNoise_Mu(M_X) = MU_MEAS_NOISE_Y;
    measNoise_Mu(M_W) = MU_MEAS_NOISE_W;
    measNoise_Mu(M_H) = MU_MEAS_NOISE_H;

    SymmetricMatrix measNoise_Cov(measSize);
    measNoise_Cov(M_X,M_X) = SIGMA_MEAS_NOISE_X;
    measNoise_Cov(M_Y,M_Y) = SIGMA_MEAS_NOISE_Y;
    measNoise_Cov(M_W,M_W) = SIGMA_MEAS_NOISE_W;
    measNoise_Cov(M_H,M_H) = SIGMA_MEAS_NOISE_H;

    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
    LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);
    // >>>>> Measures Noise

    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    // Camera Capture
    cv::VideoCapture cap;

    // >>>>> Camera Settings
    if (!cap.open(idx))
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
    // <<<<< Camera Settings

    cout << "\nHit 'q' to exit...\n";

    char ch = 0;

    double ticks = 0;
    bool found = false;

    int notFoundCount = 0;

    // >>>>> Main loop
    while (ch != 'q' && ch != 'Q')
    {
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
        cap >> frame;

        cv::Mat res;
        frame.copyTo( res );

        if (found)
        {
            // >>>> Matrix A
            //kf.transitionMatrix.at<float>(2) = dT;
            //kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            //cout << "dT:" << endl << dT << endl;

            //state = kf.predict();
            //cout << "State post:" << endl << state << endl;

            //            cv::Rect predRect;
            //            predRect.width = state.at<float>(4);
            //            predRect.height = state.at<float>(5);
            //            predRect.x = state.at<float>(0) - predRect.width / 2;
            //            predRect.y = state.at<float>(1) - predRect.height / 2;

            //            cv::Point center;
            //            center.x = state.at<float>(0);
            //            center.y = state.at<float>(1);
            //            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

            //            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
        }

        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
        // <<<<< Noise smoothing

        // >>>>> HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
        // <<<<< HSV conversion

        // >>>>> Color Thresholding
        // Note: change parameters for different colors
        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
                    cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
        // <<<<< Color Thresholding

        // >>>>> Improving the result
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        // <<<<< Improving the result

        // Thresholding viewing
        cv::imshow("Threshold", rangeRes);

        // >>>>> Contours detection
        vector<vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // <<<<< Contours detection

        // >>>>> Filtering
        vector<vector<cv::Point> > balls;
        vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
            }
        }
        // <<<<< Filtering

        cout << "Balls found:" << ballsBox.size() << endl;

        // >>>>> Detection result
        for (size_t i = 0; i < balls.size(); i++)
        {
            cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }
        // <<<<< Detection result

        // >>>>> Kalman Update
        if (balls.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                kf.statePost = state;*/
        }
        else
        {
            notFoundCount = 0;

            //            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            //            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            //            meas.at<float>(2) = (float)ballsBox[0].width;
            //            meas.at<float>(3) = (float)ballsBox[0].height;

            //            if (!found) // First detection!
            //            {
            //                // >>>> Initialization
            //                kf.errorCovPre.at<float>(0) = 1; // px
            //                kf.errorCovPre.at<float>(7) = 1; // px
            //                kf.errorCovPre.at<float>(14) = 1;
            //                kf.errorCovPre.at<float>(21) = 1;
            //                kf.errorCovPre.at<float>(28) = 1; // px
            //                kf.errorCovPre.at<float>(35) = 1; // px

            //                state.at<float>(0) = meas.at<float>(0);
            //                state.at<float>(1) = meas.at<float>(1);
            //                state.at<float>(2) = 0;
            //                state.at<float>(3) = 0;
            //                state.at<float>(4) = meas.at<float>(2);
            //                state.at<float>(5) = meas.at<float>(3);
            //                // <<<< Initialization

            //                found = true;
            //            }
            //            else
            //                kf.correct(meas); // Kalman Correction

            //            cout << "Measure matrix:" << endl << meas << endl;
        }
        // <<<<< Kalman Update

        // Final result
        cv::imshow("Tracking", res);

        // User key
        ch = cv::waitKey(1);
    }
    // <<<<< Main loop

    return EXIT_SUCCESS;
}
