#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include "../Include/Aux_OpenCV.hpp"

using namespace cv;
using namespace std;

string window_name = "OpenCV_C++_test";

int ReadCat(){
   Mat myImage;//declaring a matrix named myImage//
   namedWindow(window_name);//declaring the window to show the image//
   myImage = imread("/home/ROBOGait/Pictures/Cat03.jpg");//loading the image named lakshme in the matrix//
   if (myImage.empty()) {//If the image is not loaded, show an error message//
      cout << "Couldn't load the image." << endl;
      system("pause");//pause the system and wait for users to press any key//
      return-1;
   }
   imshow(window_name, myImage);//display the image which is stored in the 'myImage' in the "myWindow" window//  
   destroyWindow(window_name);//close the window and release allocate memory//
   waitKey(0);//wait till user press any key
   return 0;
}

int ReadWebCam(){
   //Open the default video camera
   VideoCapture cap(0);

   // if not success, exit program
   if (cap.isOpened() == false)  
   {
     cout << "Cannot open the video camera" << endl;
     cin.get(); //wait for any key press
     return -1;
   }

   //double dWidth = cap.get(CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   //double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
   //cout << "Resolution of the video : " << dWidth << " x " << dHeight << endl;

   namedWindow(window_name); //create a window called "My Camera Feed"
    
   while (true)
   {
     Mat frame;
     bool bSuccess = cap.read(frame); // read a new frame from video 

     //Breaking the while loop if the frames cannot be captured
      if (bSuccess == false) 
      {
         cout << "Video camera is disconnected" << endl;
         cin.get(); //Wait for any key press
         break;
      }

      //show the frame in the created window
      imshow(window_name, frame);

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

   return 0;
}
//return colored Canny, result is b&w Canny
Mat FullCanny(Mat* image, Mat* result, int LowThreshold, int ratio=3, int GaussKernSize=3, int SobelKernSize=3){
   Mat imageCpy;
   (*image).copyTo(imageCpy);
   //Step 1. Gaussian blurring
   Mat processedImg;                    //3x3 Kernel size
   GaussianBlur(imageCpy, processedImg, Size( GaussKernSize, GaussKernSize), 0);
   //Step 2. To Grayscale
   cvtColor(processedImg, processedImg, COLOR_BGR2GRAY);
   //Step 3. Send to Canny
   Mat cannyEdges;                              //High to low ratio of 3, recommended
   Canny(processedImg, *result, LowThreshold, LowThreshold*ratio, SobelKernSize);	
   //Ponemos a 0, negro
   processedImg = Scalar::all(0);
   //copiamos de la original poniendo solo en las zonas con bordes
   imageCpy.copyTo( processedImg, *result);
   return processedImg;
}

void CannyCallback(int LowThresh, void *userData)
{
   Mat imageCpy = *( static_cast<Mat*>(userData) );
   Mat ColorImg, BWImg;
   ColorImg = FullCanny(&imageCpy, &BWImg, LowThresh);
    
   imshow( window_name, ColorImg );
}
int CannyWebCam(){
   //Open the default video camera
   VideoCapture cap(0);

   // if not success, exit program
   if (cap.isOpened() == false)  
   {
     cout << "Cannot open the video camera" << endl;
     cin.get(); //wait for any key press
     return -1;
   }

   namedWindow(window_name);
   
   Mat frame, resized_frame;
   //create trackbar and set callback
   int trackbarVal;
   createTrackbar("Low Threshold", window_name, &trackbarVal, 100, CannyCallback, &resized_frame);
   
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
      resize(frame, resized_frame, Size(480, 480/16*9), INTER_LINEAR);
      CannyCallback(trackbarVal, &resized_frame);
      //show the frame in the created window

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

   return 0;
}
