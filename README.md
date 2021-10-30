# video.analysis

## Odometry.exe:
This is an executable to translate the keypoints detected in video to visual dots on screen. It uses the FAST corner
detection algorithm to get keypoints.

Written for OpenCV 4.5.4 on MinGW.

Usage: Odometry.exe [vidname.mp4] [prescision] [debug]
Using the [debug] option will show a frame-by-frame of the video output. Press 'Q' to quit.
The [prescision] option will set the tolerance of the FAST algorithm (Default is 30- lower values will detect more keypoints, but take longer)
