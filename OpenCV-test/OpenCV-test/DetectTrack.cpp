#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

bool show_borders=1, img_vid=true;
int lowThreshold = 0, rati=3, kernel_size=3;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		show_borders = show_borders ? false : true;
	}
}

static void CannyThreshold(int lowT, void* v)
{
	lowThreshold = lowT;
}

void FullCanny(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio=3, int GaussKSize=3, int SobelKSize = 3) {
	if (GaussKSize % 2 == 0) GaussKSize += 1;//if it's an even number, make it odd
	if (SobelKSize % 2 == 0) SobelKSize += 1;

	Mat input = img_input.getMat();
	Mat grayscale, edges;
	cvtColor(input, grayscale, COLOR_BGR2GRAY);
	GaussianBlur(grayscale, edges, Size(GaussKSize, GaussKSize), 0);
	Canny(edges, edges, _lowThreshold, _lowThreshold * _ratio, SobelKSize);
	//Because of OutputArray, we don't need a return statement
	edges.copyTo(img_output, edges);
	//imshow("FullCanny", img_output);
}

void FullCannySingleChannel(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio = 3, int GaussKSize = 3, int SobelKSize = 3) {
	if (GaussKSize % 2 == 0) GaussKSize += 1;//if it's an even number, make it odd
	if (SobelKSize % 2 == 0) SobelKSize += 1;

	Mat input = img_input.getMat();
	Mat edges;
	//if single channel we don't need grayscale
	GaussianBlur(input, edges, Size(GaussKSize, GaussKSize), 0);
	Canny(edges, edges, _lowThreshold, _lowThreshold * _ratio, SobelKSize);
	//Because of OutputArray, we don't need a return statement
	edges.copyTo(img_output, edges);
}

void FullCannyHSVInput(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio = 3, int GaussKSize = 3, int SobelKSize = 3) {
	Mat input = img_input.getMat(), inpBGR;
	try {
		cvtColor(input, inpBGR, COLOR_HSV2BGR);
	}
	catch (Exception ex) {
		cout << ex.what() << endl;
		exit(-1);
	}
	FullCanny(inpBGR, img_output, _lowThreshold, _ratio, GaussKSize, SobelKSize);
}

int main(int argc, char* argv[])
{
	Mat img;
	img = imread("E:/Visual Studio Solutions/Prueba/OpenCV-test/Track.jpg");
	//VideoCapture cap(0);
	VideoCapture cap("E:/Pigamer37/VID_20190616_214124.mp4");
	string window_name;
	if (img_vid) {

		if (img.empty())
		{
			cout << "Could not open or find the image" << endl;
			cin.get(); //wait for any key press
			return -1;
		}
		window_name = "My image";
	}
	else {
		// if not success, exit program
		if (cap.isOpened() == false)
		{
			cout << "Cannot open the video camera" << endl;
			cin.get(); //wait for any key press
			return -1;
		}
		window_name = "My Video";
	}
	namedWindow(window_name, WindowFlags::WINDOW_KEEPRATIO);
	setMouseCallback(window_name, CallBackFunc, NULL);

	createTrackbar("Min Threshold:", window_name, &lowThreshold, 100, CannyThreshold, (void*)&window_name);

	namedWindow("Control", WindowFlags::WINDOW_AUTOSIZE); //create a window called "Control"
	int iLowH = 0;
	int iHighH = 179;
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	int iLowS = 0;
	int iHighS = 255;
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	int iLowV = 250;
	int iHighV = 255;
	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	if (img_vid) {
		Mat imgCpy, morphOut, openKern, imgHSV, imgThresh;
		while (true)
		{
			img.copyTo(imgCpy);
			namedWindow("Original", WindowFlags::WINDOW_KEEPRATIO);
			imshow("Original", img);
			if (show_borders) {
				cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
				namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
				imshow("Thresholded Image", imgThresh); //show the thresholded image
				//Create a rectangular 3x3 kernel for our morphological operations
				openKern = getStructuringElement(MORPH_RECT, Size(3, 3));
				//Perform an open (dilate, then erode) with the kernel, closing missing tiny spots
				morphologyEx(imgThresh, morphOut, MORPH_OPEN, openKern);
				namedWindow("Opened Image", WindowFlags::WINDOW_NORMAL);
				imshow("Opened Image", morphOut);
				//perform a full canny algorithm (meaning passing to grayscale and blurring
				FullCannySingleChannel(morphOut, imgCpy, lowThreshold, rati, kernel_size, kernel_size);
			}
			else {
				try {
					destroyWindow("Thresholded Image");
					destroyWindow("Opened Image");
				}
				catch (Exception) {}
			}
			//imshow(window_name, imgCpy);
			imshow(window_name, imgCpy);
			if (waitKey(10) == 27)
			{
				cout << "Esc key is pressed by user. Stoppig the preview" << endl;
				break;
			}
		}
	}
	else {
		Mat frame, frameCpy;
		while (true)
		{
			bool bSuccess = cap.read(frame); // read a new frame from video 

			//Breaking the while loop if the frames cannot be captured
			if (bSuccess == false)
			{
				cout << "Video camera is disconnected" << endl;
				cin.get(); //Wait for any key press
				break;
			}
			frame.copyTo(frameCpy);
			imshow("Original", frame);
			if (show_borders) {
				FullCanny(frame, frameCpy, lowThreshold, rati, kernel_size, kernel_size);
			}
			//show the frame in the created window
			imshow(window_name, frameCpy);

			//wait for for 10 ms until any key is pressed.  
			//If the 'Esc' key is pressed, break the while loop.
			//If the any other key is pressed, continue the loop 
			//If any key is not pressed withing 10 ms, continue the loop 
			if (waitKey(10) == 27)
			{
				cout << "Esc key is pressed by user. Stoppig the video" << endl;
				break;
			}
		}
	}
	destroyAllWindows();
	return 0;
}