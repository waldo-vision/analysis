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
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace cv;

/**
 * @param[in] img_1 The image to detect keypoints from
 * @param[out] points1 Array to send keypoints to
 * @return Points of interest
 */
void featureDetection(const Mat& imgIn, std::vector<Point2f>& pointsOut, int threshold);

void featureTracking(const Mat& img_1, const Mat& img_2, std::vector<Point2f>& points1, std::vector<Point2f>& points2, std::vector<uchar>& status);

void getPoseChange(std::vector<Point2f>& firstPoints, std::vector<Point2f>& secondPoints, const double& focal, const Point2d& principalPoint, Mat& R, Mat& t);

void updateAndTrack(const Mat& firstFrame, const Mat& secondFrame, std::vector<Point2f>& firstPoints, std::vector<Point2f>& secondPoints, std::vector<uchar>& status, bool redetect = true, int threshold = 10);

int main(int argc, char** argv) {

    // Get command-line arguments
    std::cout << "Starting...\n";
    if(argc < 2) {
        std::cerr << "Missing Arg1: target";
        return 1;
    }
    String fileIn(argv[1]);
    int threshold = 10;
    if(argc >= 3) {
        threshold = std::stoi(argv[2]);
    }
    bool debug = false;
    if(argc >= 4) {
        debug = true;
    }

    // Open video
    VideoCapture cap(fileIn);
    Mat vidFrame, prevFrame;
    cap >> vidFrame;
    prevFrame = vidFrame.clone();
    // FIXME test
    int rows = vidFrame.rows;
    int cols = vidFrame.cols;
    VideoWriter vidOut;
    vidOut.open("out.mp4", VideoWriter::fourcc('m','p','4','v'), 24, vidFrame.size(), true);
    std::vector<Point2f> points, prevPoints;
    std::vector<uchar> status;
    int minGoodPoints = 2000; // TODO this is arbitrary, may need tuning (Lower threshold = higher good points needed)
    std::cout << "Video open\n";

    double focal = 100;
    Point2d opticalCenter(cols / 2, rows / 2);

    // Find t, R between first two frames
    Mat globalR, globalt;
    cap.read(prevFrame);
    cap.read(vidFrame);
    updateAndTrack(prevFrame, vidFrame, prevPoints, points, status, true, threshold);
    getPoseChange(points, prevPoints, focal, opticalCenter, globalR, globalt);


    // Main loop
    int frame = 0;
    time_t timeStart = time(NULL);
    while(cap.read(vidFrame) && vidOut.isOpened()) {        
        // Get feature movement (optical flow)
        updateAndTrack(vidFrame, prevFrame, points, prevPoints, status, prevPoints.size() < minGoodPoints, threshold);

        // Normalize feature movement RANSAC and get pose change
        Mat R, t;
        getPoseChange(points, prevPoints, focal, opticalCenter, R, t);
        // TODO test if the order of these vvv helps drift
        globalR = globalR * R;
        globalt = globalt + globalR * t;

        Mat display = vidFrame.clone();

        // Draw points to output
        for(int i = 0; i < points.size() && i < prevPoints.size(); i++) {
            line(display, points.at(i), prevPoints.at(i), Scalar::all(255), 2);
        }

        // Put this frames data into storage
        prevPoints = points;
        prevFrame = vidFrame.clone();

        // Show output (Only with debug)
        if (debug) {
            imshow("Data", display);
            char c = waitKey(0);
            if(c == 'q' || c == 'Q') break;
        }

        // Write output
        if(!debug && frame % 25 == 0) { // Make output less cluttered
            std::cout << "Frame " << frame << '\n';
        } else if(debug) {
            std::cout << "Frame " << frame << " points " << points.size() << '\n';
        }
        frame++;
        vidOut.write(display);
    }

    // Close program
    vidOut.release();
    std::cout << "Finished in " << difftime(time(NULL), timeStart) << "s.";
    return 0;
}

void featureDetection(const Mat& imgIn, std::vector<Point2f>& pointsOut, int threshold)	{ 
    bool nonmaxSuppression = true;
    std::vector<KeyPoint> keypoints(0);
    FAST(imgIn, keypoints, threshold, nonmaxSuppression);
    pointsOut.clear();
    for(KeyPoint p : keypoints) {
        pointsOut.push_back(p.pt);
    }
}

void featureTracking(const Mat& img_1, const Mat& img_2, std::vector<Point2f>& points1, std::vector<Point2f>& points2, std::vector<uchar>& status)	{ 

    //this function automatically gets rid of points for which tracking fails

    std::vector<float> err;					
    Size winSize=Size(21,21);																								
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    for(int i = 0; i < points1.size(); i++) {
        if(points1.at(i) != points2.at(i)) {
        }
    }

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

void getPoseChange(std::vector<Point2f>& firstPoints, std::vector<Point2f>& secondPoints, const double& focal, const Point2d& principalPoint, Mat& R, Mat& t) {
    // Find essential matrix
    Mat mask, E;
    E = findEssentialMat(firstPoints, secondPoints, focal, principalPoint, RANSAC, 0.999, 1.0, mask);
    // Recover pose
    recoverPose(E, firstPoints, secondPoints, R, t, focal, principalPoint, mask);
    // Remove outliers
    // TODO this will not be implimented until a recoverPose() workaround for no motion is found.
    // int removed = 0;
    // for(int i = 0; i < mask.size().area(); i++) {
    //     if(!mask.at<bool>(i)) {
    //         firstPoints.erase(firstPoints.begin() + i - removed);
    //         secondPoints.erase(secondPoints.begin() + i - removed);
    //         removed++;
    //     }
    // }
}

void updateAndTrack(const Mat& firstFrame, const Mat& secondFrame, std::vector<Point2f>& firstPoints, std::vector<Point2f>& secondPoints, std::vector<uchar>& status, bool redetect, int threshold) {
    // Grayscale frames
    Mat firstFrameG; cvtColor(firstFrame, firstFrameG, COLOR_BGR2GRAY);
    Mat secondFrameG; cvtColor(secondFrame, secondFrameG, COLOR_BGR2GRAY);
    // Find from this frame
    if(redetect) {
        featureDetection(firstFrameG, firstPoints, threshold);
        std::cout << "Redetect triggered\n";
    }
    // Track first to second
    
    if(firstFrame.at<double>(20000) == secondFrame.at<double>(20000)) std::cout << "WARNING using same frames in before tracking\n";
    featureTracking(firstFrameG, secondFrameG, firstPoints, secondPoints, status);
}