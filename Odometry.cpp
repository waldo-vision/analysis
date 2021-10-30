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
#include <iostream>
#include <cstdlib>

using namespace cv;

/**
 * @param[in] img_1 The image to detect keypoints from
 * @param[out] points1 Array to send keypoints to
 */
void featureDetection(const Mat& imgIn, std::vector<KeyPoint>& pointsOut, int threshold = 30);

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
    cap >> out;
    VideoWriter vidOut;
    VideoWriter vidOutBlank;
    vidOut.open("-out.mp4", VideoWriter::fourcc('m','p','4','v'), 24, out.size(), true);
    vidOutBlank.open("-outBlank.mp4", VideoWriter::fourcc('m','p','4','v'), 24, out.size(), true);
    std::vector<KeyPoint> keypoints;

    // Main loop
    int frame = 0;
    while(cap.read(out) && vidOut.isOpened()) {
        // Detect features
        Mat grayOut;
        int rows = out.rows;
        int cols = out.cols;
        Mat empty(rows, cols, CV_8UC3);
        cvtColor(out, grayOut, COLOR_BGR2GRAY);
        featureDetection(grayOut, keypoints, threshold);

        // Draw features
        for(KeyPoint p : keypoints) {
            drawMarker(out, Point(p.pt.x, p.pt.y), Scalar(0, 255, 0), MARKER_STAR, 1, 3);
            drawMarker(empty, Point(p.pt.x, p.pt.y), Scalar(255, 255, 255), MARKER_STAR, 1, 3);
        }

        // Show output (Only with debug)
        if (debug) {
            imshow("Data", out);
            char c = waitKey(0);
            if(c == 'q' || c == 'Q') break;
        }

        // Write output
        std::cout << "Frame " << ++frame << '\n';
        vidOut.write(out);
        vidOutBlank.write(empty);
    }

    // Close program
    vidOut.release();
    vidOutBlank.release();
    std::cout << "Reached end of video.";
    return 0;
}

void featureDetection(const Mat& imgIn, std::vector<KeyPoint>& pointsOut, int threshold)	{ 
    bool nonmaxSuppression = true;
    FAST(imgIn, pointsOut, threshold, nonmaxSuppression);
}