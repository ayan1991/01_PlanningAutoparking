#include <opencv2\opencv.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>
#include<string>
#include<stdio.h>

using namespace  cv;
using namespace  std;
template<class Type>

Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}

int main(int argc,char* argv[])
{
	//1.read motionplanning result(x,y) from txt
	ifstream  fin("in_mp_result.txt");

	string s;
	
	//float x;
	string slid = ",";
	vector<float> point;

	int x, y;
	
	const char* vehicle_img = "car.jpg";
	cv::Mat img_car = imread(vehicle_img);
	
	cv::Mat img(1500, 1500, CV_8UC3, Scalar(255, 255, 255));
	if (img_car.empty()){
		fprintf(stderr,"cannot load img %s\n",img_car);
		return -1;
	}

	cv::Rect roi_rect = cv::Rect(128, 128, img_car.cols, img_car.rows);
	img_car.copyTo(img(roi_rect));

	//°ë¾¶
	int r = 5;
	


	while (fin >> s)
	{
		if (s == ",")
		{
			continue;
		}
		point.push_back(std::stof(s));
		//cout << "Read from file: " << std::stof(s) << endl;
	}
	for (int i = 0; i < point.size(); i++)
	{
		x = 80*point[i];
		y = 80*point[i + 1];
		i = i + 1;
		Point center = Point(x, y);
		cout << "center: " << center << endl;
		circle(img, center, r, Scalar(0, 255, 0), 3);
	}

	imshow("base", img);
	waitKey();
	destroyAllWindows();
	return 0;
}