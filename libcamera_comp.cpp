//WIP

//TAKEN FROM: https://forums.raspberrypi.com/viewtopic.php?t=347925 AND 
//THE MENTIONED SIMPLECAMERA.CPP SCRIPT 
#include <libcamera/libcamera.h>
#include "event_loop.h"

using namespace libcamera;
static std::shared_ptr<Camera> camera;
static EventLoop loop;

/*
 * --------------------------------------------------------------------
 * Handle RequestComplete
 *
 * For each Camera::requestCompleted Signal emitted from the Camera the
 * connected Slot is invoked.
 *
 * The Slot is invoked in the CameraManager's thread, hence one should avoid
 * any heavy processing here. The processing of the request shall be re-directed
 * to the application's thread instead, so as not to block the CameraManager's
 * thread for large amount of time.
 *
 * The Slot receives the Request as a parameter.
 */
 
 //EN DEFINITIVA, HABRÁ QUE PASAR LA MAT CREADA AL THREAD DE PROCESAMIENTO

static void processRequest(Request *request);

static void requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	loop.callLater(std::bind(&processRequest, request));
}

static void processRequest(Request *request)
{
	const Request::BufferMap &buffers = request->buffers();
	for (auto bufferPair : buffers) {
		const Stream *stream = bufferPair.first;
		FrameBuffer *buffer = bufferPair.second;
		StreamConfiguration const &cfg = stream->configuration();
		int fd = buffer->planes()[0].fd.get();
		
	    uint8_t *ptr = static_cast<uint8_t *>(mmap(NULL, buffer->planes()[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        cv::Mat image(cfg.size.height, cfg.size.width, CV_8UC1, ptr, cfg.stride);
	}
	/* Re-queue the Request to the camera. */	
	request->reuse(Request::ReuseBuffers);
	camera->queueRequest(request);
}

//WIP: EXPECTED FUNCS

//When executed, the thread that calls this func will be the ain thread for libcamera
int StartCamera(CameraManager*& (???) cmm, Camera* cam, FrameBufferAllocator ** FBAlloc, Stream** _stream){
	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();
	
	if (cm->cameras().empty()) {
		std::cout << "No cameras were identified on the system."
			  << std::endl;
		cm->stop();
		return -1;
	}
	
	std::string cameraId = cm->cameras()[0]->id();
	cam = cm->get(cameraId);
	cam->acquire();
	
	std::unique_ptr<CameraConfiguration> config =
		cam->generateConfiguration( { StreamRole::Viewfinder } );
	StreamConfiguration &streamConfig = config->at(0);
		  
	config->validate();
	std::cout << "Validated viewfinder configuration is: "
		  << streamConfig.toString() << std::endl;
		  
	cam->configure(config.get());
	
	FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
	//WIP *FBAlloc = allocator; ???

	for (StreamConfiguration &cfg : *config) {
		int ret = allocator->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			return -1;
		}

		size_t allocated = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
	}
	
	Stream *stream = streamConfig.stream();
	//WIP *_stream = stream ???
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
	std::vector<std::unique_ptr<Request>> requests;
	for (unsigned int i = 0; i < buffers.size(); ++i) {
		std::unique_ptr<Request> request = camera->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return -1;
		}

		const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
		int ret = request->addBuffer(stream, buffer.get());
		if (ret < 0)
		{
			std::cerr << "Can't set buffer for request"
				  << std::endl;
			return -1;
		}
		
		/*
		 * Controls can be added to a request on a per frame basis.
		 */
		//ControlList &controls = request->controls();
		//controls.set(controls::Brightness, 0.5);

		requests.push_back(std::move(request));
	}
	
	cam->requestCompleted.connect(requestComplete);
	
	cam->start();
	for (std::unique_ptr<Request> &request : requests)
		cam->queueRequest(request.get());
	
	cmm = cm;
	return 0;
}
//WIP
int RunEventLoop(){
	//WIP: ESTO ES RARO, EL EVENTLOOP TAL VEZ DEBA IR EN EL MAIN? O EN UN THREAD?
		
	/*
	 * --------------------------------------------------------------------
	 * Run an EventLoop
	 *
	 * In order to dispatch events received from the video devices, such
	 * as buffer completions, an event loop has to be run.
	 */
	loop.timeout(3);
	int ret = loop.exec();
	std::cout << "Capture ran for " << 3 << " seconds and "
		  << "stopped with exit status: " << ret << std::endl;
}

//WIP
void FreeLibcamResources(CameraManager*& (???) cmm, Camera* cam, FrameBufferAllocator ** FBAlloc, Stream** _stream){
	cam->stop();
	FBAlloc->free(_stream);
	delete FBAlloc;
	cam->release();
	cam.reset();
	cmm->stop();
}
