#ifndef OCV_FUNCS_123
#define OCV_FUNCS_123
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

//constants and variables on .cpp file

void FullCanny(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio=2, int BlurKSize=3, int SobelKSize = 3);
void FullCannySingleChannel(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio = 2, int BlurKSize = 3, int SobelKSize = 3);
void FullCannyHSVInput(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio = 2, int BlurKSize = 3, int SobelKSize = 3);
void PaintCanny(InputOutputArray img_input, InputArray canny_mask, const Scalar& color, int thickness=1);
//void PaintROICanny(InputOutputArray img_input, InputArray canny_mask, int x_start, int y_start, const Scalar& color, int thickness=1);
void ApplyROI(InputOutputArray img_input, const vector<Point>& maskVerts);
void Apply_NormalROI(InputOutputArray img_input);
void Apply_Draw_ROI(InputOutputArray img_draw, InputOutputArray img_bin, const vector<Point>& maskVerts);
void Apply_Draw_NormalROI(InputOutputArray img_draw, InputOutputArray img_bin);

vector<Point2f> Get_Centroids(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area=100);
void Get_Draw_Centroids(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area=100, int roi_x=0, int roi_y=0);

void Draw_Contour_Points(InputOutputArray inp, const vector<cv::Point>& contours, const cv::Scalar& color=cv::Scalar(0, 255, 0));
void Draw_Contours_Points(InputOutputArray inp, const vector<vector<cv::Point> >& contours, const cv::Scalar& color=cv::Scalar(0, 255, 0));

vector<cv::Point> Get_One_Side_Contour(const vector<cv::Point>& contour);
vector<cv::Point> Get_Biggest_Contour(const vector<vector<cv::Point> >& contours);
vector<cv::Point> Get_Biggest_Contour_One_Line(const vector<vector<cv::Point> >& contours);
//use cv::polylines instead
void Draw_Contour(InputOutputArray inp, const vector<cv::Point>& contour, const cv::Scalar& color=cv::Scalar(155, 0, 155),int roi_x=0, int roi_y=0);
void Draw_line(InputOutputArray inp, const vector<double>& x_values, const vector<double>& y_values, const cv::Scalar& color=cv::Scalar(155, 0, 155));

void Get_Draw_Biggest_One_Line(InputOutputArray inp, const vector<vector<cv::Point> >& contours);
void Get_Draw_Biggest_Fitted_One_Line(InputOutputArray inp, const vector<vector<cv::Point> >& contours);
void Get_Draw_One_Sided_Lines(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area=100, int roi_x=0, int roi_y=0);
void Get_Draw_One_Sided_Fitted_Lines(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area=100, int roi_x=0, int roi_y=0);

void HueShift(InputArray img_hsv, OutputArray img_shift, int shift=25);
void ParallelHueShift(InputArray img_hsv, OutputArray img_shift, int shift=25);

vector<Point> SampleBinImgAtHeight(InputOutputArray img, int y);
cv::Point SampleBinImgAtHeightFirstPoint(InputOutputArray img, const cv::Point& init, const bool& direction);
vector<cv::Point> SampleBinImgAtHeightFirstPoints(InputOutputArray img, const cv::Point& init);


std::vector<int> CvPointVecExtract(const std::vector<cv::Point>& line, const bool x_or_y);
std::vector<int> CvPointVecExtractReverse(const std::vector<cv::Point>& line, const bool x_or_y);
std::vector<cv::Point> OrderByY(const std::vector<cv::Point>& line);

bool WaitForFps_or_Esc(clock_t* startTime, int delay=10);

namespace LineProcess{
	std::vector<int> GetHist(InputArray binImg, int bottomY, int topY);
	int GetHistMaxVal(const std::vector<int>& hist);
	std::vector<int> GetLocalMaxes(const std::vector<int>& hist);
	std::vector<cv::Point> GetHistLocalMaxes(InputArray binImg, int bottomY, int topY);
	void GetDrawHistLocalMaxes(InputOutputArray drawImg, InputArray binImg, int bottomY, int topY);
	void GetDrawHistLocalMaxes_Periodically(InputOutputArray drawImg, InputArray binImg, int bottomY, int topY, int window_size);
	std::vector<cv::Point> GetHistFirstLanePoints(InputArray binImg, int bottomY, int topY, int laneX=0);
	cv::Point GetHistNextLinePoint(InputArray binImg, int bottomY, int topY, int prevPointX, int thresh);
	std::vector<cv::Point> GetHistLanePoints(InputArray binImg, int bottomY, int topY, int window_size, bool leftLane, int thresh=0, int laneX=0);
	std::vector<cv::Point> CalculateMidLane(const std::vector<cv::Point>& leftLane, const std::vector<cv::Point>& rightLane);

	class laneLogic{
		int prevRightX;
		int prevLeftX;
		int prevMid;
		public:
		std::vector<cv::Point> leftLane;
		std::vector<cv::Point> rightLane;
		std::vector<cv::Point> midLane;
		laneLogic(int init_leftx=0, int init_rightx=0):prevRightX(init_rightx), prevLeftX(init_leftx){}
		void setInitPoints(int init_leftx, int init_rightx){
			this->prevRightX = init_rightx;
			this->prevLeftX = init_leftx;
		}
		void setInitPoint(int init_x){
			this->prevMid = init_x;
			this->prevRightX = init_x+100;
			this->prevLeftX = init_x-100;
		}
		std::vector<cv::Point> SmartGetHistLanePoints(InputArray binImg, int bottomY, int topY, int window_size, bool leftLane, int thresh=0){
			if(leftLane){
				std::vector<cv::Point> res;
				res = LineProcess::GetHistLanePoints(binImg, bottomY, topY, window_size, true, thresh, prevMid/*prevLeftX+100*/);
				this->prevLeftX = res[0].x;
				this->leftLane = res;
				return res;
			}else{
				std::vector<cv::Point> res;
				res = LineProcess::GetHistLanePoints(binImg, bottomY, topY, window_size, false, thresh, prevMid/*prevRightX-100*/);
			 	this->prevRightX = res[0].x;
				this->rightLane = res;
				return res;
			}
		}
		std::vector<cv::Point> CalculateMidLane(){
			std::vector<cv::Point> res = LineProcess::CalculateMidLane(this->leftLane, this->rightLane);
			prevMid = res[0].x;
			midLane = res;
			//midLane.shrink_to_fit();
			return res;
		}
	};
}

#endif
