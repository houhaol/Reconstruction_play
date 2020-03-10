
// kinect_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "pch.h"
#include <Kinect.h>      
#include <iostream>
#include <opencv2\highgui.hpp>  

using namespace std;
using namespace cv;
int main(void)
{
	IKinectSensor* mySensor = nullptr; // Get kinect Sensor
	GetDefaultKinectSensor(&mySensor);
	
	mySensor->Open(); // Open kinect sensor

	// Get depth data from kinect sensor
	IDepthFrameSource* mySource = nullptr;
	mySensor->get_DepthFrameSource(&mySource);

	// Get depth image height and width
	int height = 0, width  = 0;
	IFrameDescription* myDescription = nullptr;
	mySource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);
	myDescription->Release();
	myDescription = nullptr;

	// Get max and min depth distance
	UINT16 uDepthMin = 0, uDepthMax = 0;
	mySource->get_DepthMaxReliableDistance(&uDepthMax);
	mySource->get_DepthMinReliableDistance(&uDepthMin);
	cout << "Reliable distance:"
		<< uDepthMin << "-" << uDepthMax << endl;

	// Establish image matrix
	Mat mDepthImg(height, width, CV_16UC1);
	Mat mImg8bit(height, width, CV_8UC1);
	namedWindow("DepthImage");
	/*DepthFrame * myFrame = nullptr;
	Mat temp(height, width, CV_16UC1);
	Mat img(height, width, CV_8UC1);*/

	// Open depth data reader
	IDepthFrameReader* myReader = nullptr;
	mySource->OpenReader(&myReader);

	// Loop
	while (cv::waitKey(30) != VK_ESCAPE)
	{
		// Get the lattest image
		IDepthFrame* myFrame = nullptr;
		if (myReader->AcquireLatestFrame(&myFrame) == S_OK)
		{
			//// Save image into 16 img matrix
			//myFrame->CopyFrameDataToArray(height * width, (UINT16 *)temp.data);
			/*temp.convertTo(img, CV_8UC1, 255.0 / 4500);*/ // 4500 is the maximal distance of sensor
			myFrame->CopyFrameDataToArray(height * width,
				reinterpret_cast<UINT16*> (mDepthImg.data)); // convert data type
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax); // input args: First, output matrix; 2. conversion type; 3. scale factor;
			imshow("DepthImage", mImg8bit);
			myFrame->Release();
		}
		/*if (waitKey(30) == VK_ESCAPE) {
			break;
		}*/
	}
	myReader->Release();
	myReader = nullptr;
	mySource->Release();
	mySource = nullptr;
	mySensor->Close(); //Close sensor
	mySensor->Release();
	mySensor = nullptr;

	return 0;
}