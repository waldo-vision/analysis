/**
 * This is a program to translate the keypoints detected in video to visual dots on screen. It uses the FAST corner
 * detection algorithm to get keypoints.
 * 
 * Written for OpenCV 4.5.4 on MinGW.
 * 
 * Usage: Odometry.exe [vidname.mp4] [prescision] [debug]
 * Using the [debug] option will show a frame-by-frame of the video output. Press 'Q' to quit.
 * The [prescision] option will set the tolerance of the FAST algorithm (Default is 30- lower values will detect more keypoints, but take longer)
 */

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/Imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cstdlib>

using namespace cv;

/**
 * @param[in] img_1 The image to detect keypoints from
 * @param[out] points1 Array to send keypoints to
 * @return Points of interest
 */
std::vector<Point2f> featureDetection(const Mat& imgIn, std::vector<KeyPoint>& pointsOut, int threshold = 30);

void featureTracking(const Mat& img_1, const Mat& img_2, std::vector<Point2f>& points1, std::vector<Point2f>& points2, std::vector<uchar>& status);

int main(int argc, char** argv) {

    // Get command-line arguments
    std::cout << "Starting...\n";
    if(argc < 2) {
        std::cerr << "Missing Arg1: target";
        return 1;
    }
    String fileIn(argv[1]);
    int threshold = 30;
    if(argc >= 3) {
        threshold = std::stoi(argv[2]);
    }
    bool debug = false;
    if(argc >= 4) {
        debug = true;
    }

    // Open video
    VideoCapture cap(fileIn);
    Mat out;
    Mat grayLast;
    cap >> out;
    int rows = out.rows;
    int cols = out.cols;
    VideoWriter vidOut;
    VideoWriter mapOut; // TODO track player movement
    vidOut.open("out.mp4", VideoWriter::fourcc('m','p','4','v'), 24, out.size(), true);
    std::vector<KeyPoint> keypoints;
    std::vector<Point2f> points;
    std::vector<Point2f> prevPoints;
    std::vector<uchar> status;

    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;

    double focal = 1;
    Point2d opticalCenter(rows / 2, cols / 2);

    // Main loop
    int frame = 0;
    while(cap.read(out) && vidOut.isOpened()) {
        // Detect features
        Mat grayOut;
        cvtColor(out, grayOut, COLOR_BGR2GRAY);

        // Refresh points below a certain detection threshold (Unused, tends to track UI)
        // if(keypoints.size() < 900) {
        //     std::cout << "Points at " << keypoints.size() <<  ": refreshing\n";
        //     points = featureDetection(grayOut, keypoints, threshold);
        // }
        points = featureDetection(grayOut, keypoints, threshold);
        
        // Track features
        if(prevPoints.size() != 0) { // Not first frame
            featureTracking(grayOut, grayLast, points, prevPoints, status);

            Mat E, mask;
            std::cout << "Finding matrix\n";
            E = findEssentialMat(points, prevPoints, focal, opticalCenter, RANSAC, 0.999, 1.0, mask);
            Mat R, t;
            std::cout << "Finding pose\n";
            if(E.size().area() == 0) {
                std::cout << mask.size().area();
                std::cout << "Essential matrix is empty\n";
            } else {
                recoverPose(E, points, prevPoints, R, t, focal, opticalCenter, mask);
                // Strange output when the same image is presented
                // https://stackoverflow.com/questions/33372310/why-does-opencvs-recoverpose-return-a-non-origin-position-when-identical-point
                // Unit is (may need to test on different systems) = 0.5773502691896257
                if(*(t.begin<double>()) == 0.5773502691896257) t = 0;
                //t = t + R * t;
                sumX += t.at<double>(0);
                sumY += t.at<double>(1);
                sumZ += t.at<double>(2);
            }

            // Draw tracked features (this frame)
            for(int i = 0; i < prevPoints.size(); i++) {
                if(mask.at<bool>(i)) { // Verified point
                    // Points
                    // drawMarker(out, points[i], Scalar(255, 255, 255), MARKER_STAR, 3, 1);
                    // Tracking lines
                    line(out, points[i], prevPoints[i], Scalar(0, 100, 0), 2, LineTypes::LINE_4);
                }
            }
        }

        std::cout << "X travelled: " << sumX << " Y travelled: " << sumY << " Z travelled" << sumZ << '\n';

        // Put last frame data in storage
        prevPoints = std::vector<Point2f>(points);
        grayLast = grayOut;

        // Show output (Only with debug)
        if (debug) {
            imshow("Data", out);
            char c = waitKey(0);
            if(c == 'q' || c == 'Q') break;
        }

        // Write output
        std::cout << "Frame " << ++frame << '\n';
        vidOut.write(out);
    }

    // Close program
    vidOut.release();
    std::cout << "Reached end of video.";
    return 0;
}

std::vector<Point2f> featureDetection(const Mat& imgIn, std::vector<KeyPoint>& pointsOut, int threshold)	{ 
    bool nonmaxSuppression = true;
    FAST(imgIn, pointsOut, threshold, nonmaxSuppression);
    std::vector<Point2f> points(0);
    for(KeyPoint p : pointsOut) {
        points.push_back(p.pt);
    }
    return points;
}

void featureTracking(const Mat& img_1, const Mat& img_2, std::vector<Point2f>& points1, std::vector<Point2f>& points2, std::vector<uchar>& status)	{ 

    //this function automatically gets rid of points for which tracking fails

    std::vector<float> err;					
    Size winSize=Size(21,21);																								
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++) {
        Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) {
            if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
            }
            points1.erase (points1.begin() + i - indexCorrection);
            points2.erase (points2.begin() + i - indexCorrection);
            indexCorrection++;
        }
    }
}