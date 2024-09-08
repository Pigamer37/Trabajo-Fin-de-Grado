#include <opencv2/opencv.hpp>
//#include "opencv2/core/ocl.hpp"
#include <iostream>
#include <unistd.h>
#include <string>
//#include <pthread.h>
#include "Include/OCV_Funcs.hpp"
//#include "../RaspberryCam_Support/Include/PolyfitEigen.hpp"
#include "Include/spline.h"
#include "Include/ROSPublish.hpp"

using namespace cv;
using namespace std;

bool show_borders=1, make_ROI = 1, img_vid=false, vid_cam=true;
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
		//cout<<"Point clicked: x= "<<x<<", y= "<<y<<" end"<<endl<<endl<<endl<<endl;
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
	//ROS 2
	rclcpp::init(argc, argv);

	Mat frame;
	VideoCapture cap;
	vid_cam ? cap.open("/root/ros2_ws/src/Cam_package/src/AthTrack.webm"):cap.open(0);
	
	double fps = cap.get(CAP_PROP_FPS); //NOT FOR CAMERA
	cout<<"FPS: "<<fps<<endl;
	
	int delay = (fps>=0) ? (1000 / fps) : 10; //ms between frames based on framerate
	delay = vid_cam ? delay : 0;
	cout<<"delay (ms): "<<delay<<endl;
	cin.get();
	
	string window_name;
	if (img_vid) {
		frame = imread("/root/ros2_ws/src/Cam_package/src/Track2.webp");
		if (frame.empty())
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
	int iLowH = 130;//71
	int iHighH = 179;//133
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	int iLowS = 12;
	int iHighS = 31;
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	int iLowV = 209;
	int iHighV = 255;
	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	Mat openKern, closeKern;
	openKern = getStructuringElement(MORPH_RECT, Size(3, 3));
	closeKern = getStructuringElement(MORPH_RECT, Size(5, 5));
	//PERSPECTIVE
	vector<cv::Point2f> warpedPoints, unwarpedPoints;
	warpedPoints.reserve(4); unwarpedPoints.reserve(4);
	warpedPoints.emplace_back(565, 174); warpedPoints.emplace_back(725, 174);
	warpedPoints.emplace_back(250, 720); warpedPoints.emplace_back(1130, 720);
	warpedPoints.shrink_to_fit();
	
	unwarpedPoints.emplace_back(250, -1000); unwarpedPoints.emplace_back(1130, -1000);
	unwarpedPoints.emplace_back(250, 720); unwarpedPoints.emplace_back(1130, 720);
	unwarpedPoints.shrink_to_fit();
	Mat perspMatrix = getPerspectiveTransform(warpedPoints, unwarpedPoints);

	Mat perspFrame, frameCpy, morphOut, imgHSV, imgThresh, imgCanny;
	//ROI and laneLogic INIT
	LineProcess::laneLogic laneL;
	std::vector<cv::Point> maskVerts;
	maskVerts.reserve(4); //reserve 4 spots to avoid costly memory reallocation operations
	if(img_vid){
		laneL.setInitPoint(590);
		maskVerts.emplace_back(0, 720); maskVerts.emplace_back(1280/9, 720/3); //emplace contructs the object directly inside the vector instead of copying
		maskVerts.emplace_back(1280/9*8, 720/3); maskVerts.emplace_back(1280, 720);
	}else{
		laneL.setInitPoint(830);
		maskVerts.emplace_back(0, 720); maskVerts.emplace_back(1280/9, 720/3*2); //emplace contructs the object directly inside the vector instead of copying
		maskVerts.emplace_back(1280/9*8, 720/3*2); maskVerts.emplace_back(1280, 720);
	}
	cout<<endl;

	//ROS 2
	auto ROSPub = std::make_shared<VectorPublisher>();

	while (true)
	{
		auto startTime = getTickCount();
		if (!img_vid){
			bool bSuccess = cap.read(frame); // read a new frame from video 

			//Breaking the while loop if the frames cannot be captured
			if (bSuccess == false)
			{
				cout << "Video camera is disconnected" << endl;
				cin.get(); //Wait for any key press
				break;
			}
		}
		
		namedWindow("Original", WindowFlags::WINDOW_NORMAL);
		imshow("Original", frame);
		//PERSPECTIVE
		// warpPerspective(frame, perspFrame, perspMatrix, Size(frame.cols, frame.rows));
		// namedWindow("Perspective", WindowFlags::WINDOW_NORMAL);
		// imshow("Perspective", perspFrame);
		
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
				Apply_Draw_ROI(frameCpy, imgThresh, maskVerts);
			}
			//namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
			//imshow("Thresholded Image", imgThresh); //show the thresholded image				
			cv::morphologyEx(imgThresh, morphOut, cv::MORPH_CLOSE, closeKern);
			//cv::morphologyEx(morphOut, morphOut, cv::MORPH_OPEN, openKern);
			namedWindow("Morphed Image", WindowFlags::WINDOW_NORMAL);
			imshow("Morphed Image", morphOut);
			//Draw_Contour_Points(frameCpy, SampleBinImgAtHeightFirstPoints(morphOut, Point(morphOut.cols/2, morphOut.rows*4/5)));//SampleBinImgAtHeight(morphOut, morphOut.rows*4/5)
			//Perform Canny with medianBlur
			//FullCannySingleChannel(morphOut, imgCanny, lowThreshold, rati, kernel_size, kernel_size);
			//PaintROICanny(frameCpy, imgCanny, 0, frame.rows/3, cv::Scalar(255, 150, 0), 5);
			//PaintCanny(frameCpy, imgCanny, cv::Scalar(255, 150, 0), 5);
			//small_image.copyTo(big_image(cv::Rect(x,y,small_image.cols, small_image.rows)));
			
			//vector<vector<cv::Point> > contours;
			//cv::findContours(morphOut, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
			//drawContours(frameCpy, contours, -1, cv::Scalar(255, 0, 0), 5/*, LINE_8*/);
			//LineProcess::GetDrawHistLocalMaxes_Periodically(frameCpy, morphOut, 720, 720/3, 20);
			Draw_Contour_Points(frameCpy, /*LineProcess::*/laneL.SmartGetHistLanePoints(morphOut, 720, maskVerts[1].y, 20, true)); 
			Draw_Contour_Points(frameCpy, /*LineProcess::*/laneL.SmartGetHistLanePoints(morphOut, 720, maskVerts[1].y, 20, false));
			Draw_Contour_Points(frameCpy, /*LineProcess::*/laneL.CalculateMidLane(), cv::Scalar(150, 50, 0));
			laneL.CalculateMidLane();
			//ROS 2 publish
			ROSPub->PublishOffset(CvPointVecExtract(laneL.midLane, true), frameCpy.cols/2);

			//EIGEN
			// std::vector<int> midLanePoly = PolyFitEigenCVPoint(laneL.midLane, 2, std::vector<int>(), true);
			// std::cout<<"midlane poly: ";
			// PrintPoly(midLanePoly); std::cout<<std::endl;
			// std::cout<<std::endl;std::cout<<std::endl;std::cout<<std::endl;
			// PaintPoly(frameCpy, midLanePoly, CvPointVecExtract(laneL.midLane, true));
			// waitKey(-1);
			//SPLINE
			//std::vector<cv::Point> mid_lane = OrderByY(laneL.midLane);
			// std::vector<int> mid_y_int = CvPointVecExtractReverse(laneL.midLane, false);
			// std::vector<int> mid_x_int = CvPointVecExtractReverse(laneL.midLane, true);
			//std::vector<double> mid_x = tk::spline::CastIntVecToDouble(CvPointVecExtractReverse(laneL.midLane, true));
			//std::vector<double> mid_y = tk::spline::CastIntVecToDouble(CvPointVecExtractReverse(laneL.midLane, false));
			
			//tomamos y como eje horizontal, ya que son valores que crecen sí o sí
			//tk::spline sp(mid_y, mid_x,tk::spline::cspline_hermite);
			// for(int i=0; i<mid_y.back(); i+15){
			// 	cv::circle(frameCpy, cv::Point())
			// }
			//Draw_line(frameCpy, sp.get_y(), sp.get_x(), cv::Scalar(0, 0, 255));
			//Get_Draw_One_Sided_Fitted_Lines(frameCpy, contours, 50);
			
		}
		else {
			frame.copyTo(frameCpy);
			try {
				destroyWindow("Thresholded Image");
				destroyWindow("Morphed Image");
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
		int waitTimeMS = (delay - processTime); //time remaining to keep fps
		waitTimeMS = (waitTimeMS > 1) ? waitTimeMS : 0; //if waitTimeMS is negative or zero, we're already behind on time
		//and are going to confuse waitKey(), and if it's 1 we have to wait already, so use 0
		if(waitTimeMS!=0){ //if we had a positive non 0 or non 1 result for remaining ms...
			if (waitKey(waitTimeMS-1) == 27)//if Esc key is pressed
			{
				cout << "Esc key is pressed by user. Stoppig the video" << endl;
				//ROS 2
				rclcpp::shutdown(nullptr, "Esc key pressed");
				break;
			}
		}
		else if (waitKey(1) == 27) //obligatory for displaying purposes
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			//ROS 2
			rclcpp::shutdown(nullptr, "Esc key pressed");
			break;
		} 
		cout<<"Process Time: "<<processTime<<"(ms) "<<endl;
		cout<<"Time between frames: "<<(getTickCount()-startTime)/ getTickFrequency()*1000<<"(ms) "<<endl;
		cout<<"Actual fps: "<<(float)1/((getTickCount()-startTime)/ getTickFrequency())<<"\x1b[2;A \r";		
	}
    //THREAD
    //pthread_exit(NULL);
	destroyAllWindows();
	
	return 0;
}
