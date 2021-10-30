@echo off

echo "---Compiling %1.cpp---"
if "%1" == "" echo "Missing arg1: name of .cpp"
if "%1" == "" Exit /B

g++ -g -I"C:\OpenCV-4.5.4\install\include" -L"C:\OpenCV-4.5.4\install\x64\mingw\lib" -L"C:\OpenCV-4.5.4\install\x64\mingw\bin"  %1.cpp -lopencv_core454 -lopencv_imgproc454 -lopencv_highgui454 -lopencv_ml454 -lopencv_features2d454 -lopencv_imgcodecs454 -lopencv_videoio454 -lopencv_video454 -lopencv_calib3d454 -lopencv_dnn454 -lopencv_flann454 -lopencv_gapi454 -opencv_objdetect454 -lopencv_photo454 -lopencv_stitching454  -o %1

echo Finished! Use %1.exe to run app.
