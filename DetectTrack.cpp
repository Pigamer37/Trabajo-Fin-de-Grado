#include <opencv2/opencv.hpp>
//#include "opencv2/core/ocl.hpp"
#include <iostream>
#include <unistd.h>
#include <string>
#include <pthread.h>
#include "../RaspberryCam_Support/Include/OCV_Funcs.hpp"

using namespace cv;
using namespace std;

bool show_borders=1, make_ROI = 1, img_vid=false, vid_cam=true, GPU_Processing = true;
int lowThreshold = 0, rati=2, kernel_size=3;

//uso con GPU
//cuda::GpuMat in_GPU, out_GPU;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		show_borders = show_borders ? false : true;
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		make_ROI = make_ROI ? false : true;
		cout<<"Point clicked: x= "<<x<<", y= "<<y<<" end"<<endl<<endl<<endl<<endl;
	}
}

static void CannyThreshold(int lowT, void* v)
{
	lowThreshold = lowT;
}

static void CannyKSize(int kS, void* v)
{
	kernel_size = kS;
}

//THREAD
void *myThreadFunction(void *arg) {
    // thread code here#include "opencv2/core/ocl.hpp"
    return NULL;
} 

int main(int argc, char* argv[])
{
	//checkOpenCL();
	//ocl::setUseOpenCL(true);
	//checkOpenCL();
	
	Mat img;
	VideoCapture cap;
	vid_cam ? cap.open("/home/ROBOGait/Pictures/AthTrack.webm"):cap.open(0);
	
	//THREAD
	//pthread_t myThread;
    //pthread_create(&myThread, NULL, myThreadFunction, NULL);
	
	double fps = cap.get(CAP_PROP_FPS); //NOT FOR CAMERA
	cout<<"FPS: "<<fps<<endl;
	
	int delay = (fps>=0) ? (1000 / fps) : 10; //ms between frames based on framerate
	delay = vid_cam ? delay : 0;
	cout<<"delay (ms): "<<delay<<endl;
	cin.get();
	
	string window_name;
	if (img_vid) {
		img = imread("/home/ROBOGait/Documents/TFG Lucas GV/Track2.webp");
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
	namedWindow(window_name, WindowFlags::WINDOW_NORMAL);
	setMouseCallback(window_name, CallBackFunc, NULL);

	createTrackbar("Min Threshold:", window_name, &lowThreshold, 100, CannyThreshold, (void*)&window_name);
	createTrackbar("SobelKernelSize:", window_name, &kernel_size, 7, CannyKSize);

	namedWindow("Control", WindowFlags::WINDOW_AUTOSIZE); //create a window called "Control"
	int iLowH = 71;
	int iHighH = 133;
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	int iLowS = 0;
	int iHighS = 36;
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	int iLowV = 205;
	int iHighV = 255;
	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	if (img_vid) {
		Mat imgCpy, morphOut, openKern, imgHSV, imgThresh, imgCanny;
		while (true)
		{
			img.copyTo(imgCpy);
			namedWindow("Original", WindowFlags::WINDOW_NORMAL);
			imshow("Original", img);
			if (show_borders) {
				cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
				namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
				imshow("Thresholded Image", imgThresh); //show the thresholded image
				//Create a rectangular 3x3 kernel for our morphological operations
				openKern = getStructuringElement(MORPH_RECT, Size(7, 7));
				//Perform an open (dilate, then erode) with the kernel, closing missing tiny spots
				morphologyEx(imgThresh, morphOut, MORPH_CLOSE, openKern);
				namedWindow("Closed Image", WindowFlags::WINDOW_NORMAL);
				imshow("Closed Image", morphOut);
				//Perform Canny with medianBlur
				FullCannySingleChannel(morphOut, imgCanny, lowThreshold, rati, kernel_size, kernel_size);
				PaintCanny(imgCpy, imgCanny, Scalar(255, 150, 0), 5);
				
				//vector<vector<Point> > contours;
				//findContours(imgCanny, contours, RetrievalModes::RETR_LIST, ContourApproximationModes::CHAIN_APPROX_SIMPLE);
				//drawContours(imgCpy, contours, -1, Scalar(255, 0, 0), 5);
			}
			else {
				try {
					destroyWindow("Thresholded Image");
					destroyWindow("Closed Image");
					//destroyWindow("Canny Image");
				}
				catch (Exception) { ; }
			}
			imshow(window_name, imgCpy);
			
			if (waitKey(10) == 27)
			{
				cout << "Esc key is pressed by user. Stoppig the preview" << endl;
				break;
			}
		}
	}
	else {
		Mat frame, perspFrame, frameCpy, morphOut, openKern, closeKern, imgHSV, imgThresh, imgCanny;
		//Create a rectangular 3x3 kernel for our morphological operations
		openKern = getStructuringElement(MORPH_RECT, Size(3, 3));
		closeKern = getStructuringElement(MORPH_RECT, Size(5, 5));
		//PERSPECTIVE
		vector<cv::Point2f> warpedPoints, unwarpedPoints;
		warpedPoints.reserve(4); unwarpedPoints.reserve(4);
		warpedPoints.emplace_back(835, 693); warpedPoints.emplace_back(974, 693);
		warpedPoints.emplace_back(713, 1078); warpedPoints.emplace_back(1156, 1078);
		warpedPoints.shrink_to_fit();
		
		unwarpedPoints.emplace_back(713, 250); unwarpedPoints.emplace_back(1156, 250);
		unwarpedPoints.emplace_back(713, 1078); unwarpedPoints.emplace_back(1156, 1078);
		unwarpedPoints.shrink_to_fit();
		Mat perspMatrix = getPerspectiveTransform(warpedPoints, unwarpedPoints);
		
		cout<<endl;
		while (true)
		{
			auto startTime = getTickCount();
			bool bSuccess = cap.read(frame); // read a new frame from video 

			//Breaking the while loop if the frames cannot be captured
			if (bSuccess == false)
			{
				cout << "Video camera is disconnected" << endl;
				cin.get(); //Wait for any key press
				break;
			}
			
			namedWindow("Original", WindowFlags::WINDOW_NORMAL);
			imshow("Original", frame);
			//PERSPECTIVE
			warpPerspective(frame, perspFrame, perspMatrix, Size(frame.cols, frame.rows));
			namedWindow("Perspective", WindowFlags::WINDOW_NORMAL);
			imshow("Perspective", perspFrame);
			
			if (show_borders) {
				resize(frame, frame, Size(1280, 720), 0, 0, INTER_CUBIC);
			//NEW
				frame.copyTo(frameCpy);
				cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
				ParallelHueShift(imgHSV, imgHSV, 45); //Shifts hue circle by (shift*2)º
				cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
				if(make_ROI){
					//std::cout<<"Entered ROI drawing"<<std::endl;
					//std::cout<<"Entered ROI aplication"<<std::endl;
					Apply_Draw_NormalROI(frameCpy, imgThresh);
				}
				//namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
				//imshow("Thresholded Image", imgThresh); //show the thresholded image				
				cv::morphologyEx(imgThresh, morphOut, cv::MORPH_CLOSE, closeKern);
				//cv::morphologyEx(morphOut, morphOut, cv::MORPH_OPEN, openKern);
				namedWindow("Closed Image", WindowFlags::WINDOW_NORMAL);
				imshow("Closed Image", morphOut);
				Draw_Contour_Points(frameCpy, SampleBinImgAtHeight(morphOut, morphOut.rows*4/5));//*4/5
				//Perform Canny with medianBlur
				//FullCannySingleChannel(morphOut, imgCanny, lowThreshold, rati, kernel_size, kernel_size);
				//PaintROICanny(frameCpy, imgCanny, 0, frame.rows/3, cv::Scalar(255, 150, 0), 5);
				//PaintCanny(frameCpy, imgCanny, cv::Scalar(255, 150, 0), 5);
				//small_image.copyTo(big_image(cv::Rect(x,y,small_image.cols, small_image.rows)));
				
				vector<vector<cv::Point> > contours;
				cv::findContours(morphOut, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
				//drawContours(frameCpy, contours, -1, cv::Scalar(255, 0, 0), 5/*, LINE_8*/);
				Get_Draw_Centroids(frameCpy, contours, 50);
				Get_Draw_One_Sided_Fitted_Lines(frameCpy, contours, 50);
			//OLD
				//frame.copyTo(frameCpy);
				////FullTrackPass(frame, frameCpy, true);
				//cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
				//ParallelHueShift(imgHSV, imgHSV, 25); //Shifts hue circle by (shift*2)º
				//inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
				//namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
				//imshow("Thresholded Image", imgThresh); //show the thresholded image				
				////Perform an open (dilate, then erode) with the kernel, closing missing tiny spots
				////morphologyEx(imgThresh, morphOut, MORPH_OPEN, openKern);
				//morphologyEx(imgThresh, morphOut, MORPH_CLOSE, openKern);
				//namedWindow("Closed Image", WindowFlags::WINDOW_NORMAL);
				//imshow("Closed Image", morphOut);
				////Perform Canny with medianBlur
				//FullCannySingleChannel(morphOut, imgCanny, lowThreshold, rati, kernel_size, kernel_size);
				//frame.copyTo(frameCpy);
				//PaintCanny(frameCpy, imgCanny, Scalar(255, 150, 0), 5);
				
				////vector<vector<Point> > contours;
				////findContours(morphOut, contours, RetrievalModes::RETR_LIST, ContourApproximationModes::CHAIN_APPROX_SIMPLE);
				////drawContours(frameCpy, contours, -1, Scalar(255, 0, 0), 5);
			}
			else {
				frame.copyTo(frameCpy);
				try {
					destroyWindow("Thresholded Image");
					destroyWindow("Closed Image");
					//destroyWindow("Canny Image");
				}
				catch (Exception) { ; }
			}
			//show the frame in the created window
			imshow(window_name, frameCpy);
			
			//wait for for 10 ms until any key is pressed.  
			//If the 'Esc' key is pressed, break the while loop.
			//If the any other key is pressed, continue the loop 
			//If any key is not pressed withing 10 ms, continue the loop 
			//if (waitKey(10) == 27)
			//{
				//cout << "Esc key is pressed by user. Stoppig the video" << endl;
				//break;
			//}
			int processTime = (getTickCount() - startTime)/ getTickFrequency()*1000; //processing time it took since the beggining of the frame loop
			cout<<"\x1b[1;A \x1b[1;A \rProcess Time: "<<processTime<<"(ms) "<<endl;
			int waitTimeMS = (delay - processTime); //time remaining to keep fps
			waitTimeMS = (waitTimeMS > 1) ? waitTimeMS : 0; //if waitTimeMS is negative or zero, we're already behind on time
			//and are going to confuse waitKey(), and if it's 1 we have to wait already, so use 0
			if(waitTimeMS!=0){ //if we had a positive non 0 or non 1 result for remaining ms...
				if (waitKey(waitTimeMS-1) == 27)//if Esc key is pressed
				{
					cout << "Esc key is pressed by user. Stoppig the video" << endl;
					break;
				}
			}
			else if (waitKey(1) == 27) //obligatory for displaying purposes
			{
				cout << "Esc key is pressed by user. Stoppig the video" << endl;
				break;
			} 
			cout<<"Time between frames: "<<(getTickCount()-startTime)/ getTickFrequency()*1000<<"(ms) "<<endl;
			cout<<"Actual fps: "<<(float)1/((getTickCount()-startTime)/ getTickFrequency());		
		}
	}
    //THREAD
    //pthread_exit(NULL);
	destroyAllWindows();
	return 0;
}
