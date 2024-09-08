#include <iostream>
#include <signal.h>

#include <string>
//#include "Include/OCV_Funcs.hpp"

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
using namespace cv;

static std::shared_ptr<Camera> camera;

bool recording=0, preview=1;
cv::Mat frame, frameCpy;
cv::VideoWriter video;

///New try
std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		recording = recording ? false : true;
	}
	else if (event == cv::EVENT_MBUTTONDOWN)
	{
        if(preview){
            cv::destroyWindow("Preview");
            preview = false;
            recording = true;
        }else preview = true;
	}
}

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
	return;
	//FPS
	//auto startTime = cv::getTickCount();
	
	const Request::BufferMap &buffers = request->buffers();
	//const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
	//std::cout<<"Buffer number: "<<buffers.size()<<std::endl;
	for (auto bufferPair : buffers) {
		//FPS
		//startTime = getTickCount();
		
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
        
        if(recording && !preview){
            video.write(frame);
        }
        else if(preview){
            frame.copyTo(frameCpy);
            if(recording){
                putText(frameCpy, "Recording", cv::Point(5,50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 3, cv::LINE_8, false);
                video.write(frame);
            }
            imshow("Preview", frameCpy);
        }
	}
	/* Re-queue the Request to the camera. */
	request->reuse(Request::ReuseBuffers);
	camera->queueRequest(request);
}


int main(int argc, char** argv)
{
    cv::String keys =
        "{help h usage ? |      | to run with preview input preview, to record from the start input record   }"
        "{preview        |      | boolean describing if the program should start with an image preview/GUI }"         // optional, default value ""
        "{record         |      | boolean describing if the program should start recording instantly }";       // optional, default value ""

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // cv::String input_image_path = parser.get<cv::String>(0); // read @image (mandatory, error if not present)
    // cv::String face_cascade_path = parser.get<cv::String>(1); // read @face (use default value if not in cmd)

    preview = parser.has("preview");
    recording = parser.has("record");

    // cv::String eye_cascade_path = parser.get<cv::String>("eyes"); 
    // cv::String nose_cascade_path = parser.get<cv::String>("nose"); 
    // cv::String mouth_cascade_path = parser.get<cv::String>("mouth");

    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }


	std::string window_name = "Preview";
	cv::namedWindow(window_name, WindowFlags::WINDOW_NORMAL);
	cv::setMouseCallback(window_name, CallBackFunc, NULL);
	//cv::namedWindow("Original", cv::WindowFlags::WINDOW_NORMAL);
	
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

    //VideoWriter config
    video.open("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(1280, 720));
    if (!video.isOpened())
    {
        std::cout  << "Could not open the output video for write: " << std::endl;
        return -1;
    }

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
	   
	std::cout<<std::endl;
//////Loop//////////////////////////////////////////////////////////////   
	//int ret=0;
	//do{
		//ret = cv::waitKey(1);
	//}while (ret!=27);
	if (cv::waitKey(0) == 27) //obligatory for displaying purposes
	{
			std::cout <<std::endl<< "Esc key is pressed by user. Stoppig the video" << std::endl;
			//break;
	}
	   
//////EndLoop///////////////////////////////////////////////////////////
    video.release();
	cv::destroyAllWindows();
	std::cout<<"OpenCV windows destroyed"<<std::endl
	<<"Attempting to stop camera"<<std::endl;
	//kill(0, 6);
    camera->stop();
	camera->requestCompleted.disconnect(requestComplete);
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
