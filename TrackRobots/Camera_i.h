#ifndef ICAMERA_H_
#define ICAMERA_H_

#include <cv.h>
#include <highgui.h>
#include <vector>

#include "UserConfig.h"
#include "GlobalStructsDef.h"

using namespace cv;
using namespace std;

/* VisibleRobot - another robot detected by the camera */
class VisibleRobot : public CamMesuredRobot
{
public:
	VisibleRobot(Point2d centroid, Rect bBox, Rect sBox);
	~VisibleRobot();

	bool isRobotInsideBox();
	void setRobotParams(Point2d centroid, Rect bBox, Rect sBox);

protected:
	Point2d m_centroid;
	Rect m_bBox; /* bounding box */
	Rect m_sBox; /* search box */
	// TODO - KalmanFilter  kalmanFilter;
};

class Camera_i
{
protected:
	Mat m_frame_RGB;
	Mat m_frame_GRAY;
	vector<VisibleRobot*> m_visibleRobots;  /* vector of visible robots */
	unsigned int numFrames;
	unsigned int currentFrameInex;

	void removeSmallBlobs(cv::Mat& im, double size);
	void imadjust(Mat& frame, double contrast, int brightness);
	Stats regionStats(const vector<vector<Point> > contours, const int idx, const bool addRobots);
	bool isRobot(const Stats stats);
	void create_sBox(const Stats stats, Rect& sBox);
	void drawDetection(const Stats stats, const Rect sBox);

public:
	bool cameraInit();
	bool captureFrame();
	bool isReDetectionRequired();
	Mat  detectColor(Mat& frameHSV);                 /* returns imborder */
	bool detectTable(Mat imborder, int attempts);   /* returns true if succeded. Changes m_frame to hold table only */
	void detectRobots(int addRobots);				/* Detects Robots*/
};

#endif /* ICAMERA_H_ */