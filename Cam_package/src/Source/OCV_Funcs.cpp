
#include "../Include/OCV_Funcs.hpp"

//default values for some functions on .hpp file

void FullCanny(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio, int BlurKSize, int SobelKSize) {
	if (BlurKSize % 2 == 0) BlurKSize += 1;//if it's an even number, make it odd
	if (SobelKSize % 2 == 0) SobelKSize += 1;

	Mat input = img_input.getMat().clone();
	Mat grayscale, edges;
	cvtColor(input, grayscale, COLOR_BGR2GRAY);
	medianBlur(grayscale, edges, BlurKSize);
	Canny(edges, edges, _lowThreshold, _lowThreshold * _ratio, SobelKSize);
	//Because of OutputArray, we don't need a return statement
	edges.copyTo(img_output);
	//imshow("FullCanny", img_output);
}

void FullCannySingleChannel(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio, int BlurKSize, int SobelKSize) {
	if (BlurKSize % 2 == 0) BlurKSize += 1;//if it's an even number, make it odd
	if (SobelKSize % 2 == 0) SobelKSize += 1;

	Mat input = img_input.getMat().clone();
	Mat edges(input.rows, input.cols, CV_8UC1, Scalar::all(0));;
	//if single channel we don't need grayscale
	medianBlur(input, edges, BlurKSize);
	try {
		Canny(edges, edges, _lowThreshold, _lowThreshold * _ratio, SobelKSize);
		edges.copyTo(img_output);
		//Because of OutputArray, we don't need a return statement
	}
	catch (Exception& ex) {
		cout <<"FullCannySingleChannel: "<< ex.what() << endl;
		exit(-1);
	}
}

void FullCannyHSVInput(InputArray img_input, OutputArray img_output, int _lowThreshold, int _ratio, int BlurKSize, int SobelKSize) {
	Mat input = img_input.getMat(), inpBGR;
	try {
		cvtColor(input, inpBGR, COLOR_HSV2BGR);
	}
	catch (Exception& ex) {
		cout << ex.what() << endl;
		exit(-1);
	}
	FullCanny(inpBGR, img_output, _lowThreshold, _ratio, BlurKSize, SobelKSize);
}

void PaintCanny(InputOutputArray img_input, InputArray canny_mask, const Scalar& color, int thickness){
	Mat Color(img_input.getMat().rows, img_input.getMat().cols, CV_8UC3, color);
	Mat dilated_canny;
	if(thickness<1) thickness=1;
	if(thickness!=1) {
		Mat dilateKern = getStructuringElement(MORPH_RECT, Size(thickness, thickness));
		dilate(canny_mask, dilated_canny, dilateKern);
	}
	else dilated_canny = canny_mask.getMat().clone();
	Color.copyTo(img_input, canny_mask);
}

void ApplyROI(InputOutputArray img_input, const vector<Point>& maskVerts){
	cv::Mat mask(img_input.getMat().rows, img_input.getMat().cols, CV_8UC1, Scalar(0));
	cv::fillPoly(mask, maskVerts, 255);
	bitwise_and(img_input, mask, img_input);
}

void Apply_NormalROI(InputOutputArray img_input){
	vector<cv::Point> maskVerts;
	Mat inp = img_input.getMat();
	//ROI Mask
	maskVerts.reserve(4); //reserve 4 spots to avoid costly memory reallocation operations
	maskVerts.emplace_back(0, inp.rows); maskVerts.emplace_back(inp.cols/9, inp.rows/1.7f); //emplace contructs the object directly inside the vector instead of copying
	maskVerts.emplace_back(inp.cols/9*8, inp.rows/1.7f); maskVerts.emplace_back(inp.cols, inp.rows);
	ApplyROI(inp, maskVerts);
}
//img_draw and img_bin must have the same size, maskverts is the contour of the ROI
void Apply_Draw_ROI(InputOutputArray img_draw, InputOutputArray img_bin, const vector<Point>& maskVerts){
	Draw_Contour(img_draw.getMat(), maskVerts, Scalar(155, 155, 0));
	ApplyROI(img_bin.getMat(), maskVerts);
}
//img_draw and img_bin must have the same size
void Apply_Draw_NormalROI(InputOutputArray img_draw, InputOutputArray img_bin){
	vector<cv::Point> maskVerts;
	Mat inp = img_draw.getMat();
	//ROI Mask
	maskVerts.reserve(4); //reserve 4 spots to avoid costly memory reallocation operations
	maskVerts.emplace_back(0, inp.rows); maskVerts.emplace_back(inp.cols/9, inp.rows/1.7f);
	maskVerts.emplace_back(inp.cols/9*8, inp.rows/1.7f); maskVerts.emplace_back(inp.cols, inp.rows);
	
	Apply_Draw_ROI(inp, img_bin.getMat(), maskVerts);
}

//void PaintROICanny(InputOutputArray img_input, InputArray canny_mask, int x_start, int y_start, const Scalar& color, int thickness){
	//Mat Color(img_input.getMat().rows, img_input.getMat().cols, CV_8UC3, color);
	//Mat dilated_canny;
	//if(thickness<1) thickness=1;
	//if(thickness!=1) {
		//Mat dilateKern = getStructuringElement(MORPH_RECT, Size(thickness, thickness));
		//dilate(canny_mask, dilated_canny, dilateKern);
	//}
	//else dilated_canny = canny_mask.getMat().clone();
	//Mat littleimg_input = img_input.getMat()(cv::Rect(x_start,y_start,Color.cols, Color.rows));
	//Color.copyTo(littleimg_input, canny_mask);
	//littleimg_input.copyTo(img_input.getMat()(cv::Rect(x_start,y_start,Color.cols, Color.rows)));
//}

vector<Point2f> Get_Centroids(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area){
	vector<Moments> mu( contours.size() );
	vector<Point2f> mc( contours.size() );
	vector<Point2f> mret;
	
	for( size_t i = 0; i < contours.size(); i++ )
	{
		mu[i] = moments( contours[i] );
		//add 1e-5 to avoid division by zero
		if(mu[i].m00 > min_area){
			mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
			static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) );
			mret.push_back(mc[i]);
		}
	}
	return mret;
}

void Get_Draw_Centroids(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area, int roi_x, int roi_y){
	
	vector<Moments> mu( contours.size() );
	vector<Point2f> mc( contours.size() );
	vector<cv::Point> shifted_cont;
	
	for( size_t i = 0; i < contours.size(); i++ )
	//for (const vector<cv::Point> single_cont : contours)
	{
		shifted_cont.reserve(contours[i].size()); //size before to reduce reallocation
		for(size_t j = 0; j < contours[i].size(); j++){
			shifted_cont[j].x = contours[i][j].x + roi_x;
			shifted_cont[j].y = contours[i][j].y + roi_y;
		}
		mu[i] = moments( contours[i] ); //contours[i] shifted
		//add 1e-5 to avoid division by zero
		if(mu[i].m00 > min_area){
			mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
			static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) );
			//cout << "mc[" << i << "]=" << mc[i] << endl;
			circle( inp, mc[i], 10, cv::Scalar(0, 0, 255), -1 );
		}
		shifted_cont.clear();
	}
}

void Draw_Contour_Points(InputOutputArray inp, const vector<cv::Point>& contour, const cv::Scalar& color){
	//for( size_t j = 0; j < contour.size(); j++ ){ //recorremos los puntos de cada contorno
	for(const cv::Point& point : contour){
		circle( inp, point, 10, color, -1);
	}
}

void Draw_Contours_Points(InputOutputArray inp, const vector<vector<cv::Point> >& contours, const cv::Scalar& color){
	//for( size_t i = 0; i < contours.size(); i++ ){ //recorremos los contornos de la imagen
	for (const vector<cv::Point>& contour : contours){
		Draw_Contour_Points(inp, contour, color); //contours[i]
	}
}

vector<cv::Point> Get_One_Side_Contour(const vector<cv::Point>& contour){
	vector<cv::Point> one_side_dir1, one_side_dir2;
	cv::Point prev_point = contour[0];
	cv::Point current_point;
	int lowest_Point_Index = 0;
	
	for(size_t i = 1; i < contour.size(); i++ ){
		current_point = contour[i];
		if(current_point.y < prev_point.y ){
			prev_point = current_point;
			lowest_Point_Index = i;
		} //si encontramos  un punto del contorno más bajo, lo guardamos
	}
	one_side_dir1.reserve(contour.size()-(lowest_Point_Index)); //reservamos el espacio necesario previamente
	one_side_dir2.reserve(lowest_Point_Index+1);
	one_side_dir1.push_back(prev_point); //Incluimos el punto más bajo en el contorno nuevo
	one_side_dir2.push_back(prev_point);
	
	if(lowest_Point_Index+1<contour.size()){
		for(size_t i = lowest_Point_Index+1; i < contour.size(); i++ ){//recorremos desde el punto más bajo en una dirección
			current_point = contour[i];
			if(current_point.y>=prev_point.y){ //si vamos subiendo añadimos a la línea
				one_side_dir1.push_back(current_point);
				prev_point = current_point;
			}else break; //si dejamos de subir, paramos
		}
	}
	if(lowest_Point_Index-1 > 0){
		for(size_t i = lowest_Point_Index-1; i > 0; i-- ){//recorremos desde el punto más bajo en la otra dirección
			current_point = contour[i];
			if(current_point.y>=prev_point.y){ //si vamos subiendo añadimos a la línea
				one_side_dir2.push_back(current_point);
				prev_point = current_point;
			}else break; //si dejamos de subir, paramos
		}
	}
	
	if(one_side_dir1.size()>=one_side_dir2.size()) return one_side_dir1;
	else return one_side_dir2;
}

vector<cv::Point> Get_Biggest_Contour(const vector<vector<cv::Point> >& contours){
	Moments m1, m2;
	vector<cv::Point> biggest_area_contour = contours[0];
	m1 = moments( contours[0] );
	//for( size_t i = 1; i < contours.size(); i++ ){
	for (const vector<cv::Point>& single_cont : contours){
		m2 = moments( single_cont );
		if(m2.m00 > m1.m00){ //si el area es mayor
			biggest_area_contour = single_cont;
			m1 = moments( biggest_area_contour );
		}
	}
	return biggest_area_contour;
}

vector<cv::Point> Get_Biggest_Contour_One_Line(const vector<vector<cv::Point> >& contours){
	vector<cv::Point> biggest_area_contour = Get_Biggest_Contour(contours);
	return Get_One_Side_Contour(biggest_area_contour);
}

void Draw_Contour(InputOutputArray inp, const vector<cv::Point>& contour, const Scalar& color, int roi_x, int roi_y){
	vector<cv::Point> shifted_cont;
	shifted_cont.reserve(contour.size()); //allocate necessary space from the start
	shifted_cont[0].x = contour[0].x + roi_x;
	shifted_cont[0].y = contour[0].y + roi_y;
	for(int i = 0; i < contour.size()-1; i++){
		shifted_cont[i+1].x = contour[i+1].x + roi_x;
		shifted_cont[i+1].y = contour[i+1].y + roi_y;
		cv::line(inp, shifted_cont[i], shifted_cont[i+1], color, 5);
	}
}

void Draw_line(InputOutputArray inp, const vector<double>& x_values, const vector<double>& y_values, const cv::Scalar& color){
	std::cout<<"inside drawlines";
	for(int i = 0; i < x_values.size()-1; i++){
		cv::line(inp, cv::Point(x_values[i], y_values[i]), cv::Point(x_values[i+1], y_values[i+1]), color, 5);
	}
}

void Get_Draw_Biggest_One_Line(InputOutputArray inp, const vector<vector<cv::Point> >& contours){
	vector<cv::Point> one_sided = Get_Biggest_Contour_One_Line(contours);
	std::cout<<"Got biggest contour. Size: "<<one_sided.size()<<std::endl;
	if (one_sided.size()<2){return;}
	//for(int i = 0; i < one_sided.size()-1; i++){
		//cv::line(inp, one_sided[i], one_sided[i+1], Scalar(155, 0, 155), 2);
	//}
	Draw_Contour(inp, one_sided);
}
void Get_Draw_Biggest_Fitted_One_Line(InputOutputArray inp, const vector<vector<cv::Point> >& contours){
	vector<cv::Point> one_sided = Get_Biggest_Contour_One_Line(contours);
	std::cout<<"Got biggest contour"<<std::endl;
	vector<cv::Point> one_sided_simple;
	cv::approxPolyDP(one_sided, one_sided_simple, 0.1, false);
	Draw_Contour(inp, one_sided_simple);
}

void Get_Draw_One_Sided_Lines(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area, int roi_x, int roi_y){
	vector<Moments> mu( contours.size() );
	
	for( size_t i = 1; i < contours.size(); i++ ){
	//for (const vector<cv::Point>& single_cont : contours){
		const vector<cv::Point>& single_cont = contours[i];
		mu[i] = moments( single_cont ); //contours[i]

		if(mu[i].m00 > min_area){ //filtro por área
			Draw_Contour(inp, Get_One_Side_Contour(single_cont), roi_x, roi_y);
		}
	}
}

void Get_Draw_One_Sided_Fitted_Lines(InputOutputArray inp, const vector<vector<cv::Point> >& contours, int min_area, int roi_x, int roi_y){
	vector<Moments> mu( contours.size() );
	
	for( size_t i = 1; i < contours.size(); i++ ){
	//for (const vector<cv::Point>& single_cont : contours){
		const vector<cv::Point>& single_cont = contours[i];
		mu[i] = moments( single_cont );

		if(mu[i].m00 > min_area){ //filtro por área
			vector<cv::Point> one_sided_simple;
			cv::approxPolyDP(Get_One_Side_Contour(single_cont), one_sided_simple, 0.1, false);
			Draw_Contour(inp, one_sided_simple);
		}
	}
}

void HueShift(InputArray img_hsv, OutputArray img_shift, int shift){
	std::vector<cv::Mat> channels;
	split(img_hsv.getMat(), channels);

	Mat &H = channels[0];
	Mat &S = channels[1];
	Mat &V = channels[2];
	
	Mat shiftedH = H.clone();
	for(int j=0; j<shiftedH.rows; ++j)
		for(int i=0; i<shiftedH.cols; ++i)
		{// in openCV hue values go from 0 to 180 (so have to be doubled to get to 0 .. 360) because of byte range from 0 to 255
			shiftedH.at<unsigned char>(j,i) = (shiftedH.at<unsigned char>(j,i) + shift)%180;
		}
	//namedWindow("NormalH", WindowFlags::WINDOW_NORMAL);
	//imshow("NormalH", H);
	H = shiftedH.clone();
	//namedWindow("ShiftedH", WindowFlags::WINDOW_NORMAL);
	//imshow("ShiftedH", H);
	merge(channels, img_shift);
}

void ParallelHueShift(InputArray img_hsv, OutputArray img_shift, int shift){
	std::vector<cv::Mat> channels;
	split(img_hsv.getMat(), channels);

	Mat &H = channels[0];
	Mat &S = channels[1];
	Mat &V = channels[2];
	
	Mat shiftedH = H.clone();
	//optimization (reduced performance by aprox 10ms, from aprox. 85ms to 75ms)
	parallel_for_(Range(0, shiftedH.rows*shiftedH.cols), [&](const Range& range){
		for (int r = range.start; r < range.end; r++)
		{
			int j = r / shiftedH.cols;
            int i = r % shiftedH.cols;
			shiftedH.at<unsigned char>(j,i) = (shiftedH.at<unsigned char>(j,i) + shift)%180;
		}
	});
	
	shiftedH.copyTo(H);
	merge(channels, img_shift);
}
//las y van hacia abajo en OpenCV
vector<cv::Point> SampleBinImgAtHeight(InputOutputArray img, int y){
	Mat inp = img.getMat();
	vector<cv::Point> positives;
	for(int i = 0; i<inp.cols; i++){
		if((int)(inp.at<uchar>(y, i)) == 255){
			positives.emplace_back(i, y);
		}
	}
	return positives;
}

cv::Point SampleBinImgAtHeightFirstPoint(InputOutputArray img, const cv::Point& init, const bool& direction){
	Mat inp = img.getMat();
	int y = init.y;
	
	if(direction){
		for(int i = init.x; i<inp.cols; i++){
			if((int)(inp.at<uchar>(y, i)) == 255){
				return Point(i, y);
			}
		}
		return Point(0,0);
	}else{
		for(int i = init.x; i>=0; i--){
			if((int)(inp.at<uchar>(y, i)) == 255){
				return Point(i, y);
			}
		}
		return Point(0,0);
	}
}

vector<cv::Point> SampleBinImgAtHeightFirstPoints(InputOutputArray img, const cv::Point& init){
	std::vector<cv::Point> points;
	points.reserve(2);
	points[0] = SampleBinImgAtHeightFirstPoint(img, init, false);
	points[1] = SampleBinImgAtHeightFirstPoint(img, init, true);
	return points;
}

//DOESN'T ESCAPE, DON'T USE
bool WaitForFps_or_Esc(clock_t* startTime, int delay){
	clock_t processTime = clock() - *startTime;
	while (processTime < delay) {
		if (waitKey((int)(delay - processTime)) == 27)//if Esc key is pressed
		{
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			return true;
		}
		processTime = clock() - *startTime; //update processTime just in case
	}
	return false;
}

std::vector<int> CvPointVecExtract(const std::vector<cv::Point>& line, const bool x_or_y){
	std::vector<int> coordinates;
	coordinates.reserve(line.size());
	if(x_or_y){
		for(int i=0; i<line.size(); i++){
			coordinates.emplace_back(line[i].x);
		}
	}else{
		for(int i=0; i<line.size(); i++){
			coordinates.emplace_back(line[i].y);
		}
	}
	return coordinates;
}

std::vector<int> CvPointVecExtractReverse(const std::vector<cv::Point>& line, const bool x_or_y){
	std::vector<int> coordinates;
	coordinates.reserve(line.size());
	if(x_or_y){
		for(int i=(line.size()-1); i>=0; i--){
			coordinates.emplace_back(line[i].x);
		}
	}else{
		for(int i=(line.size()-1); i>=0; i--){
			coordinates.emplace_back(line[i].y);
		}
	}
	return coordinates;
}

std::vector<cv::Point> OrderByY(const std::vector<cv::Point>& line){
    int n = line.size();
	std::vector<cv::Point> line2 = line;
    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j < n - i - 1; ++j) {
            if (line2[j].y < line2[j + 1].y) {
                std::swap(line2[j], line2[j + 1]);
            }
        }
    }
	return line2;
}


namespace LineProcess{
	std::vector<int> GetHist(InputArray binImg, int bottomY, int topY, int thresh){
		if(bottomY<topY){ //si nos lo dan al revés le damos la vuelta 
			int temp = bottomY;
			bottomY = topY;
			topY = temp;
		}
		
		cv::Mat inp = binImg.getMat();//topX, topY, width, height
		int height = (bottomY-topY);
		cv::Mat subImg = inp(cv::Rect(0, topY, (inp.cols), height));
		
		std::vector<int> hist;
		hist.reserve(inp.cols);
		int value = 0;
		
		for(int j = 0; j<(inp.cols); j++){ //recorrer columnas
			value = 0;
			for(int i = 0; i<height; i++){//recorrer cada fila de la columna
				if((int)(subImg.at<uchar>(i, j)) != 0){ value++;}
			}
			if(value>=thresh){
				hist.emplace_back(value);
			}
			else hist.emplace_back(0);
		}

		return hist;
	}
	int GetHistMaxVal(const std::vector<int>& hist){
		int max = 0;
		for(int val:hist){
			if(val>max){
				max = val;
			}
		}
		return max;
	}
	std::vector<int> GetLocalMaxes(const std::vector<int>& hist){
		int local_max = 0;
		bool up = true;
		int local_max_index_first = 0;
		std::vector<int> local_max_indexes;
		
		for(int i = 0; i<hist.size(); i++){
			if(hist[i] > local_max){
				up = true;
				local_max = hist[i];
				local_max_index_first = i;
			}else if(hist[i] < local_max){
				if(up){
					up = false;
					local_max_indexes.push_back((i+local_max_index_first)/2); //si se mantiene un valor en varias columnas seguidas, calcula la media
				}
				local_max = hist[i];
			}
		}
		return local_max_indexes;
	}
	std::vector<cv::Point> GetHistLocalMaxes(InputArray binImg, int bottomY, int topY){
		int midY = (bottomY+topY)/2;
		std::vector<int> hist = GetHist(binImg, bottomY, topY, 5);
		
		std::vector<int> local_max_indexes = GetLocalMaxes(hist);
		// std::cout<<local_max_indexes.size()<<" is the number of local maximums"<<std::endl;
		// cv::waitKey(-1);
		
		std::vector<cv::Point> local_max_points;
		local_max_points.reserve(local_max_indexes.size());
		for(int i = 0; i<local_max_indexes.size(); i++){
			local_max_points.emplace_back(local_max_indexes[i], midY);
		}
		return local_max_points;
	}
	void GetDrawHistLocalMaxes(InputOutputArray drawImg, InputArray binImg, int bottomY, int topY){
		Draw_Contour_Points(drawImg.getMat(), GetHistLocalMaxes(binImg, bottomY, topY));
	}
	void GetDrawHistLocalMaxes_Periodically(InputOutputArray drawImg, InputArray binImg, int bottomY, int topY, int window_size){
		if(bottomY<topY){ //si nos lo dan al revés le damos la vuelta 
			int temp = bottomY;
			bottomY = topY;
			topY = temp;
		}
		
		const int height = bottomY - topY;
		int n_of_windows = (int)height / window_size;
		
		int current_bottomY = bottomY;
		
		for(int i=0; i<n_of_windows; i++){
			GetDrawHistLocalMaxes(drawImg.getMat(), binImg.getMat(), current_bottomY, (current_bottomY-window_size));
			current_bottomY -= window_size;
		}
		//if(height % window_size != 0) GetDrawHistLocalMaxes(drawImg, binImg, bottomY, current_topY);
	}
	std::vector<cv::Point> GetHistFirstLanePoints(InputArray binImg, int bottomY, int topY, int laneX){
		if(bottomY<topY){ //si nos lo dan al revés le damos la vuelta 
			int temp = bottomY;
			bottomY = topY;
			topY = temp;
		}

		int searchPoint = (laneX<=0)?binImg.getMat().cols/2:laneX;

		std::vector<cv::Point> points = GetHistLocalMaxes(binImg, bottomY, topY);
		std::vector<cv::Point> lanePoints;
		lanePoints.reserve(2);

		if(points.size()==0){
			lanePoints.emplace_back(-1,-1);
			lanePoints.emplace_back(-1,-1);
			return lanePoints;
		}

		int leftLaneIdx = 0, rightLaneIdx = 0;
		int currentLeftMinDist = -searchPoint;
		int currentRightMinDist = searchPoint;
		for(int i = 1; i<points.size(); i++){
			int pointDist = points[i].x-searchPoint;

			if(pointDist>0){ //possible right lane
				if(pointDist<currentRightMinDist){
					currentRightMinDist = pointDist;
					rightLaneIdx = i;
				}
			}else{ //posible left lane
				if(pointDist>currentLeftMinDist){
					currentLeftMinDist = pointDist;
					leftLaneIdx = i;
				}
			}
		}

		lanePoints.push_back(points[leftLaneIdx]);
		lanePoints.push_back(points[rightLaneIdx]);

		return lanePoints;
	}
	cv::Point GetHistNextLinePoint(InputArray binImg, int bottomY, int topY, int prevPointX, int thresh){
		if(bottomY<topY){ //si nos lo dan al revés le damos la vuelta 
			int temp = bottomY;
			bottomY = topY;
			topY = temp;
		}

		std::vector<cv::Point> points = GetHistLocalMaxes(binImg, bottomY, topY);
		cv::Point lanePoint;

		if(points.size()==0){
			lanePoint = cv::Point(-1,-1);
			return lanePoint;
		}

		int laneIdx = 0;
		thresh = std::abs(thresh);
		int currentMinDist = thresh;
		for(int i = 1; i<points.size(); i++){
			int pointDist = std::abs(points[i].x-prevPointX);
			if(pointDist<currentMinDist){
				currentMinDist = pointDist;
				laneIdx = i;
			}
		}
		if(currentMinDist==thresh){ //no ha registrado una distancia menor
			lanePoint = cv::Point(-1,-1);
			return lanePoint;
		}

		lanePoint = points[laneIdx];

		return lanePoint;
	}
	std::vector<cv::Point> GetHistLanePoints(InputArray binImg, int bottomY, int topY, int window_size, bool leftLane, int thresh, int laneX){
		if(bottomY<topY){ //si nos lo dan al revés le damos la vuelta 
			int temp = bottomY;
			bottomY = topY;
			topY = temp;
		}
		
		const int height = bottomY - topY;
		int n_of_windows = (int)height / window_size;

		int current_bottomY = bottomY-window_size;
		std::vector<cv::Point> laneSeeds = GetHistFirstLanePoints(binImg, bottomY, current_bottomY, laneX);
		cv::Point seedPoint, nextPoint;
		std::vector<cv::Point> lane;
		//detectar error
		if(laneSeeds[0]==cv::Point(-1, -1)){
			lane.emplace_back(-1,-1);
			return lane;
		}
		//elegir izq o der
		if(leftLane){ seedPoint = laneSeeds[0]; }
		else if(laneSeeds.size()>=2){ seedPoint = laneSeeds[1]; }
		else {
			lane.emplace_back(-1,-1);
			return lane;
		}
		lane.push_back(seedPoint);

		if(thresh<=0) thresh = window_size*2;

		for(int i=1; i<n_of_windows; i++){
			nextPoint = GetHistNextLinePoint(binImg, current_bottomY, (current_bottomY-window_size), seedPoint.x, thresh);
			if(nextPoint!=cv::Point(-1, -1)){ //point is valid, add and update
				lane.push_back(nextPoint);
				seedPoint = nextPoint;
			}//else lane.emplace_back(seedPoint.x, current_bottomY-(window_size/2));
			else return lane; //discard and stop
			//if we didn't get a valid point, use seedpoint again
			current_bottomY -= window_size; //go to the next window
		}
		return lane;
	}
	std::vector<cv::Point> CalculateMidLane(const std::vector<cv::Point>& leftLane, const std::vector<cv::Point>& rightLane){
		int limit = (leftLane.size()<=rightLane.size())?leftLane.size():rightLane.size();
		std::vector<cv::Point> midLane;
		midLane.reserve(limit);
		int mid_x;
		for(int i=0; i<limit; i++){
			mid_x = (leftLane[i].x+rightLane[i].x)/2;
			midLane.emplace_back(mid_x, leftLane[i].y);
		}
		std::cout<<"midlane:y, then x"<<std::endl;
				for(int i=0; i<midLane.size(); i++){
					std::cout<<midLane[i].y<<", ";
				}
				std::cout<<std::endl;
				for(int i=0; i<midLane.size(); i++){
					std::cout<<midLane[i].x<<", ";
				}
				std::cout<<std::endl;
		return midLane;
	}
}
