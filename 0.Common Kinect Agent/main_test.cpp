#include "kinectagent.h"

int main() {
	KinectAgent kinect;

	kinect.initColorSensor().getColorImage();
	return 0;
}