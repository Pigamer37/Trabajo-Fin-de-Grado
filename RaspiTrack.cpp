#include <iostream>
#include <signal.h>

#include <string>
#include "Include/OCV_Funcs.hpp"

#include <iomanip>
#include <memory>
#include <sys/mman.h>

//#include "/usr/include/libcamera/libcamera/libcamera.h"
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>

/*
 * Some code from https://github.com/edward-ardu/libcamera-cpp-demo/tree/main
 * */

using namespace libcamera;

static std::shared_ptr<Camera> camera;

bool show_borders=1, make_ROI = 0;
int lowThreshold = 0, rati=2, kernel_size=3;
cv::Mat frame, frameCpy, roi, morphOut, imgHSV, imgThresh, imgCanny, openKern = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));;
//Create a rectangular 3x3 kernel for our morphological operations

///New try
std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;

int iLowH = 0; //71
int iHighH = 179; //133
int iLowS = 0;
int iHighS = 36;
int iLowV = 205;
int iHighV = 255;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		show_borders = show_borders ? false : true;
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		make_ROI = make_ROI ? false : true;
	}
}

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
	return;
	//FPS
	auto startTime = cv::getTickCount();
	
	const Request::BufferMap &buffers = request->buffers();
	//const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
	//std::cout<<"Buffer number: "<<buffers.size()<<std::endl;
	for (auto bufferPair : buffers) {
		//FPS
		startTime = getTickCount();
		
		const Stream *stream = bufferPair.first;
		FrameBuffer *buffer = bufferPair.second;
		StreamConfiguration const &cfg = stream->configuration();
		uint8_t* imgData;
		//std::vector<cv::Mat> channels;
		for (unsigned int i = 0; i < buffer->planes().size(); i++)
		{	
			//New try
			const FrameBuffer::Plane &plane = buffer->planes()[i];
			const FrameMetadata::Plane &meta = buffer->metadata().planes()[i];
			
			void *data = mappedBuffers_[plane.fd.get()].first;
			imgData = (uint8_t *)data;
			int length = std::min(meta.bytesused, plane.length);
			
			//uint8_t *ptr = static_cast<uint8_t *>(mmap(NULL, buffer->planes()[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, buffer->planes()[i].fd.get(), 0));
			//channels.push_back( cv::Mat(cfg.size.height, cfg.size.width, CV_8U, ptr, cfg.stride) );
			
		}
		cv::Mat frame(cfg.size.height, cfg.size.width, CV_8UC3, imgData, cfg.stride);
		//merge(channels, frame);
		//framingo.copyTo(frame);
		//cv::imshow("Original", frame);
			
		if (show_borders) {
			//resize(frame, frame, Size(1280, 720), 0, 0, INTER_CUBIC);
			//roi = frame(Rect(0, frame.rows/3, frame.cols, frame.rows));
			frame.copyTo(frameCpy);
			cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
			ParallelHueShift(imgHSV, imgHSV, 25); //Shifts hue circle by (shift*2)º
			cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
			if(make_ROI){
				//std::cout<<"Entered ROI drawing"<<std::endl;
				//std::cout<<"Entered ROI aplication"<<std::endl;
				Apply_Draw_NormalROI(frameCpy, imgThresh);
			}
			//namedWindow("Thresholded Image", WindowFlags::WINDOW_NORMAL);
			//imshow("Thresholded Image", imgThresh); //show the thresholded image				
			cv::morphologyEx(imgThresh, morphOut, cv::MORPH_CLOSE, openKern);
			//cv::morphologyEx(morphOut, morphOut, cv::MORPH_OPEN, openKern);
			namedWindow("Closed Image", WindowFlags::WINDOW_NORMAL);
			imshow("Closed Image", morphOut);
			SampleBinImgAtHeight(morphOut, 1.7f);
			//Perform Canny with medianBlur
			//FullCannySingleChannel(morphOut, imgCanny, lowThreshold, rati, kernel_size, kernel_size);
			//PaintROICanny(frameCpy, imgCanny, 0, frame.rows/3, cv::Scalar(255, 150, 0), 5);
			//PaintCanny(frameCpy, imgCanny, cv::Scalar(255, 150, 0), 5);
			//small_image.copyTo(big_image(cv::Rect(x,y,small_image.cols, small_image.rows)));
			
			vector<vector<cv::Point> > contours;
			cv::findContours(morphOut, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
			cv::drawContours(frameCpy, contours, -1, cv::Scalar(255, 0, 0), 2/*, LINE_8*/);

			Get_Draw_Centroids(frameCpy, contours, 50);
			
			//Get_Draw_Biggest_One_Line(frameCpy, contours);
			//Get_Draw_One_Sided_Lines(frameCpy, contours, 50);
			
		}
		else {
			frame.copyTo(frameCpy);
			try {
				cv::destroyWindow("Thresholded Image");
				cv::destroyWindow("Closed Image");
				//destroyWindow("Canny Image");
			}
			catch (Exception) { ; }
		}
		cv::imshow("My camera", frameCpy);
		
		//cv::waitKey(0);
		
		int processTime = (cv::getTickCount() - startTime)/ cv::getTickFrequency()*1000; //processing time it took since the beggining of the frame loop
		std::cout<<"\x1b[1;A \x1b[1;A \rProcess Time: "<<processTime<<"(ms) "<<std::endl;
		//int waitTimeMS = (delay - processTime); //time remaining to keep fps
		//waitTimeMS = (waitTimeMS > 1) ? waitTimeMS : 0; //if waitTimeMS is negative or zero, we're already behind on time
		////and are going to confuse waitKey(), and if it's 1 we have to wait already, so use 0
		//if(waitTimeMS!=0){ //if we had a positive non 0 or non 1 result for remaining ms...
			//if (waitKey(waitTimeMS-1) == 27)//if Esc key is pressed
			//{
				//cout << "Esc key is pressed by user. Stoppig the video" << endl;
				//break;
			//}
		//}
		 
		std::cout<<"Time between frames: "<<(cv::getTickCount()-startTime)/ cv::getTickFrequency()*1000<<"(ms) "<<std::endl;
		std::cout<<"Actual fps: "<<(float)1/((cv::getTickCount()-startTime)/ cv::getTickFrequency());
	    
	}
	/* Re-queue the Request to the camera. */
	request->reuse(Request::ReuseBuffers);
	camera->queueRequest(request);
}


int main()
{
	string window_name = "My camera";
	cv::namedWindow(window_name, WindowFlags::WINDOW_NORMAL);
	cv::setMouseCallback(window_name, CallBackFunc, NULL);
	//cv::namedWindow("Original", cv::WindowFlags::WINDOW_NORMAL);
	
	cv::namedWindow("Control", cv::WindowFlags::WINDOW_AUTOSIZE); //create a window called "Control"
	//int iLowH = 71;
	//int iHighH = 133;
	cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &iHighH, 179);
	//int iLowS = 0;
	//int iHighS = 36;
	cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &iHighS, 255);
	//int iLowV = 205;
	//int iHighV = 255;
	cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &iHighV, 255);
	
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();
	
	for (auto const &camera : cm->cameras())
		std::cout << camera->id() << std::endl;
		
	if (cm->cameras().empty()) {
		std::cout << "No cameras were identified on the system."
				  << std::endl;
		cm->stop();
		return -1;
	}

	std::string cameraId = cm->cameras()[0]->id();
	camera = cm->get(cameraId);
	
	camera->acquire();
	
	std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Viewfinder } );
	StreamConfiguration &streamConfig = config->at(0);
	config->at(0).pixelFormat = formats::RGB888;
	std::cout << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;
	streamConfig.size.width = 1280; //800
	streamConfig.size.height = 720; //600
	config->validate();
	std::cout << "Validated viewfinder configuration is: " << streamConfig.toString() << std::endl;
	camera->configure(config.get());
	
	FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration &cfg : *config) {
		int ret = allocator->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			return -ENOMEM;
		}

		size_t allocated = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
	}
	Stream *stream = streamConfig.stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
	std::vector<std::unique_ptr<Request>> requests;
	
	for (unsigned int i = 0; i < buffers.size(); ++i) {
		std::unique_ptr<Request> request = camera->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return -ENOMEM;
		}

		const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
		int ret = request->addBuffer(stream, buffer.get());
		if (ret < 0)
		{
			std::cerr << "Can't set buffer for request"
				  << std::endl;
			return ret;
		}
		/////////////////New try
		for (const FrameBuffer::Plane &plane : buffer->planes()) {
			void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
			mappedBuffers_[plane.fd.get()] = std::make_pair(memory, plane.length);
		}
//////////////////
		requests.push_back(std::move(request));
	}
	
	camera->requestCompleted.connect(requestComplete);
	
	camera->start();
	for (std::unique_ptr<Request> &request : requests)
	   camera->queueRequest(request.get());
	   
	cout<<endl;
//////Loop//////////////////////////////////////////////////////////////   
	//int ret=0;
	//do{
		//ret = cv::waitKey(1);
	//}while (ret!=27);
	if (cv::waitKey(0) == 27) //obligatory for displaying purposes
	{
			cout <<std::endl<< "Esc key is pressed by user. Stoppig the video" << endl;
			//break;
	}
	   
//////EndLoop///////////////////////////////////////////////////////////

	cv::destroyAllWindows();
	std::cout<<"OpenCV windows destroyed"<<std::endl
	<<"Attempting to stop camera"<<std::endl;
	//kill(0, 6);
	camera->requestCompleted.disconnect(requestComplete);
	camera->stop();
	std::cout<<"Cammera stopped"<<std::endl;
	
	for (auto &iter : mappedBuffers_)
	{
		std::pair<void *, unsigned int> pair_ = iter.second;
		munmap(std::get<0>(pair_), std::get<1>(pair_));
	}
	//}
	mappedBuffers_.clear();
	std::cout<<"Buffers cleared"<<std::endl;
	
	allocator->free(stream);
	delete allocator;
	std::cout<<"Allocator deleted"<<std::endl;
	camera->release();
	camera.reset();
	cm->stop();
	cm.reset();
	std::cout<<"All memory deallocated"<<std::endl;
	exit(0);
}
