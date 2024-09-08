#ifndef AUX_OPENCV_123
#define AUX_OPENCV_123
#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

//shows Cat image in path
int ReadCat();
//reads default webcam and displays result
int ReadWebCam();
//Callback for Canny threshold trackbar
Mat FullCanny(Mat* image, Mat* result, int LowThreshold, int ratio, int GaussKernSize, int SobelKernSize);
void CannyCallback(int LowThresh, void *userData);
//Full Canny applied to webcam feed
int CannyWebCam();

#endif
