#include "kinectagent.h"

KinectAgent::KinectAgent() {
	//Connect to kinect
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	std::cerr << "Connecting to kinect" << std::endl;
	if (FAILED(hr)) {
		std::cerr << "Can't find kinect!" << std::endl;
		return;
	}

	//Open the kinect sensor
	if (m_pKinectSensor)
		hr = m_pKinectSensor->Open();
	if (SUCCEEDED(hr))
		return;
	else std::cerr << "Open sensor failed!" << std::endl;
}

KinectAgent::~KinectAgent() {
	std::cerr << "Shutting down sensor." << std::endl;
	//Empty matrix
	if (m_pColorRGBX) {
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
	if (m_pDepthRGBX) {
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	//Disconnect kinect
	if (m_pKinectSensor)
		m_pKinectSensor->Close();

	SafeRelease(m_pKinectSensor);
	std::cerr << "Sensor closed." << std::endl;
}


void KinectAgent::initColorSensor(void) {
	//Create matrix
	m_pColorRGBX = new RGBQUAD[CAP_COLOR_WIDTH * CAP_COLOR_HEIGHT];
	m_cvColorMat = cv::Mat::zeros(CAP_COLOR_SIZE, CV_8UC3);

	//Create frame source
	IColorFrameSource* pColorFrameSource = NULL;

	//Open color sensor source
	hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	if (SUCCEEDED(hr))
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	if (SUCCEEDED(hr))
		std::cerr << "Color sensor opened." << std::endl;
	else std::cerr << "Open color sensor failed." << std::endl;

	SafeRelease(pColorFrameSource);
}

void KinectAgent::initDepthSensor() {
	//Create matrix
	m_pDepthRGBX = new RGBQUAD[CAP_DEPTH_WIDTH * CAP_DEPTH_HEIGHT];
	m_cvDepthMat = cv::Mat::zeros(CAP_DEPTH_SIZE, CV_8UC3);

	//Create frame source
	IDepthFrameSource* pDepthFrameSource = NULL;

	//Open depth sensor source
	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	if (SUCCEEDED(hr))
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	if (SUCCEEDED(hr))
		std::cerr << "Depth sensor opened." << std::endl;
	else std::cerr << "Open depth sensor failed." << std::endl;

	SafeRelease(pDepthFrameSource);
}

cv::Mat KinectAgent::getColorImage() {
	//Create containers
	UINT nColorBufferSize = 0;
	RGBQUAD* pColorBuffer = NULL;
	IColorFrame* pColorFrame = NULL;
	ColorImageFormat imageFormat = ColorImageFormat_None;

	//Grab latest image
	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	//Convert image to raw data
	if (SUCCEEDED(hr)) {
		hr = pColorFrame->get_RawColorImageFormat(&imageFormat);

		if (SUCCEEDED(hr)) {

			//Copy raw data to buffer
			if (imageFormat == ColorImageFormat_Bgra) {
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			} else if (m_pColorRGBX) {
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = CAP_COLOR_WIDTH * CAP_COLOR_HEIGHT * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			} else hr = E_FAIL;

			//Convert buffer data to OpenCV matrix
			m_cvColorMat = ConvertMat(pColorBuffer, CAP_COLOR_WIDTH, CAP_COLOR_HEIGHT);
		}
		SafeRelease(pColorFrame);
	}
	return m_cvColorMat;
}

cv::Mat KinectAgent::getDepthImage() {
	UINT nDepthBufferSize = 0;
	UINT16* pDepthBuffer = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IFrameDescription* pFrameDescription = NULL;
	USHORT nMinDepth = 0; //DepthMinReliableDistance
	USHORT nMaxDepth = 0; //DepthMaxDistance

	hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr)) {

		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		if (SUCCEEDED(hr))
			hr = pDepthFrame->get_DepthMinReliableDistance(&nMinDepth);
		if (SUCCEEDED(hr)) {
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nMaxDepth = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}
		if (SUCCEEDED(hr)) {
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		if (SUCCEEDED(hr)) {
			RGBQUAD* pRGBX = m_pDepthRGBX;

			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pDepthBuffer + (CAP_COLOR_WIDTH * CAP_DEPTH_HEIGHT);

			while (pDepthBuffer < pBufferEnd) {
				USHORT depth = *pDepthBuffer;

				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).

				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

				pRGBX->rgbRed = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue = intensity;

				++pRGBX;
				++pDepthBuffer;
			}

		}


	}

}