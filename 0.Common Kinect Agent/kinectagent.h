#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <Kinect.h>

//Color sensor: 1920x1080
const int CAP_COLOR_WIDTH = 1920;
const int CAP_COLOR_HEIGHT = 1080;
const cv::Size CAP_COLOR_SIZE(CAP_COLOR_WIDTH, CAP_COLOR_HEIGHT);

//Depth sensor: 512x424
const int CAP_DEPTH_WIDTH = 512;
const int CAP_DEPTH_HEIGHT = 424;
const cv::Size CAP_DEPTH_SIZE(CAP_DEPTH_WIDTH, CAP_DEPTH_HEIGHT);


class KinectAgent {

	HRESULT hr;

	//Kinect Device
	IKinectSensor* m_pKinectSensor = NULL;

	//Sensors
	IColorFrameReader*  m_pColorFrameReader = NULL;
	IDepthFrameReader*  m_pDepthFrameReader = NULL;

	//Image matrix
	RGBQUAD* m_pColorRGBX = NULL;
	RGBQUAD* m_pDepthRGBX = NULL;
	cv::Mat m_cvColorMat;
	cv::Mat m_cvDepthMat;

	//Convert kinect image to OpenCV format
	inline cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight) {
		const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

		//Create a matrix container
		cv::Mat tmp(nHeight, nWidth, CV_8UC3);
		uchar* pMatrixData = tmp.data;

		//Copy BGR buffer data to matrix
		while (pBuffer < pBufferEnd) {
			*pMatrixData = pBuffer->rgbBlue;
			pMatrixData++;
			*pMatrixData = pBuffer->rgbGreen;
			pMatrixData++;
			*pMatrixData = pBuffer->rgbRed;
			pMatrixData++;

			++pBuffer;
		}
		return tmp;
	}

	//Release memory space
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease) {
		if (pInterfaceToRelease != NULL) {
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

public:
	KinectAgent(void);
	~KinectAgent(void);

	void initColorSensor(void);
	void initDepthSensor(void);

	cv::Mat getColorImage(void);
	cv::Mat getDepthImage(void);

};

