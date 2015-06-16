#include "Camera_i.h"



VisibleRobot::VisibleRobot(Point2d centroid, Rect bBox, Rect sBox)
	: m_centroid(centroid), m_bBox(bBox), m_sBox(sBox)
{

}

VisibleRobot::~VisibleRobot()
{
}

void VisibleRobot::setRobotParams(Point2d centroid, Rect bBox, Rect sBox)
{
	m_centroid = centroid;
	m_bBox = bBox;
	m_sBox = sBox;
}

bool Camera_i::cameraInit()
{
	numFrames = 0;
	currentFrameInex = 0;
	#if (VIDEO_INPUT == CAMERA)

		//default capture width and height
		const int FRAME_WIDTH = 1280;
		const int FRAME_HEIGHT = 1024;

		// init camera
		//HIDS hCam = (HIDS) 0;				// open next camera
		//is_InitCamera (&hCam, NULL);		// init camera - no window handle required	

		//if ( hCam != 0 )
		//{
			/* capture picture */
		//	if( is_FreezeVideo( hCam, IS_WAIT ) == IS_SUCCESS )
				/* display picture */
			//	is_RenderBitmap( hCam, 0, NULL, IS_RENDER_NORMAL );
		//}
		//open capture object at location zero (default location for webcam)
		videoCapture.open(0);
		//set height and width of capture frame
		videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
		videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	#else
		bool isOpen = videoCapture.open("..\\..\\4Robots_3.avi");
		numFrames = (unsigned int)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
	#endif

	return true;
}

void Camera_i::imadjust(Mat& frame, double contrast, int brightness)
{
	Mat new_image = Mat::zeros( frame.size(), frame.type() );

	// Do the operation new_image(i,j) = alpha*image(i,j) + beta
	for( int y = 0; y < frame.rows; y++ )
	{ 
		for( int x = 0; x < frame.cols; x++ )
		{ 
			for( int c = 0; c < frame.channels(); c++ )
			{
					new_image.at<Vec3b>(y,x)[c] =
						saturate_cast<uchar>( contrast * ( frame.at<Vec3b>(y,x)[c] ) + brightness );
			}
		}
	}

	frame = new_image;
 }


void Camera_i::removeSmallBlobs(cv::Mat& im, double size)
	/**
 * Replacement for Matlab's bwareaopen()
 * Input image must be 8 bits, 1 channel, black and white (objects)
 * with values 0 and 255 respectively
 */
{
    // Only accept CV_8UC1
    if (im.channels() != 1 || im.type() != CV_8U)
        return;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = cv::contourArea(contours[i]);

        // Remove small objects by drawing the contour with black color
        if (area > 0 && area <= size)
            cv::drawContours(im, contours, i, CV_RGB(0,0,0), -1);
    }
}


bool Camera_i::captureFrame()
{
	bool endvideoflag = false;
	//store image to matrix
	videoCapture.read(m_frame_RGB);
	cvWaitKey(100);		/* get frame from camera */	
	//imshow("CapturedImage", m_frame_RGB);
	
	

	//int numFrames = (int) cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);
		
	/* offline- recording */
	#if (VIDEO_INPUT == AVI_FILE)
	{
		currentFrameInex++;
		if(currentFrameInex >= numFrames)
		{
			endvideoflag = true;
		}
	}
	#endif

	cout << "\rindexFrames= " << currentFrameInex;

	return endvideoflag;

}


bool Camera_i::isReDetectionRequired()
{
	static bool isFirst = true;
	if (isFirst == true)
	{
		isFirst = false;
		return true;
	}
	else
	{
		return false;
	}
}


Mat Camera_i::detectColor(Mat& frameHSV)
{
	Mat thresholdImage, m_frame_RGB_resized ;
	resize(m_frame_RGB, m_frame_RGB_resized, Size(),  1 / systemParameters.scale, 1 / systemParameters.scale);
	//imshow("resized RGB frame",m_frame_RGB_resized);

	imadjust(m_frame_RGB_resized, hsvParameters.contrastValue, hsvParameters.brightnessValue);
    //imshow("adjusted RGB frame",m_frame_RGB_resized);

	//convert frame from BGR to HSV colorspace
	cvtColor(m_frame_RGB_resized, frameHSV, COLOR_BGR2HSV);
	//imshow("frameHSV", frameHSV);

	//filter HSV image between values and store filtered image to threshold matrix
	inRange(frameHSV, Scalar(hsvParameters.hsvColors[H_MIN], hsvParameters.hsvColors[S_MIN], hsvParameters.hsvColors[V_MIN]),
		Scalar(hsvParameters.hsvColors[H_MAX], hsvParameters.hsvColors[S_MAX], hsvParameters.hsvColors[V_MAX]), thresholdImage);
	
	// imshow("Thresholded Image", thresholdImage);

	return thresholdImage;
}


bool Camera_i::detectTable(Mat imborder, int attempts)
{
	Mat bw_dilate, bw_inv, bw_fill, tableMask;
	Rect filledSize;
	/* init Rect struct */
	filledSize.x = 0;
	filledSize.y = 0;
	filledSize.width = imborder.cols;
	filledSize.height = imborder.rows;

	bool tableFound = false;
	int attemptsCount = 0;

	/* Choose morphological structuring element */
	if(attempts == 1)
	{
		// DO_NOTHING. Use default morfological element 
	}
	else
	{
		/* Create bigger morphological structuring element */
		// TODO conversion from 'double' to 'int', possible loss of data
		int coefficient = (int) (((1 + 0.5 * attempts) * TABLE_MORPH_ELEMENT_SIZE)/ systemParameters.scale);
		Size ksize = Size(coefficient, coefficient); 
		detectionParameters.seTable = getStructuringElement(MORPH_ELLIPSE , ksize);
	}
	
	try
	{
		/* Dilate image */
		dilate(imborder, bw_dilate, detectionParameters.seTable);
		//imshow("bw_dilate", bw_dilate);

		/* Image inverse */
		bitwise_not( bw_dilate, bw_inv );
		//imshow("bw_inv", bw_inv);

		/* Fill holes 
		* bw_dilate - input image,  Point(bw_dilate.cols/2 ,bw_dilate.rows/2) – Starting point: image center
		* 255 - New value of the repainted domain pixels (White color)
		* filledSize Optional output parameter set by the function to the minimum bounding rectangle of the repainted domain
		*/
		floodFill( bw_dilate, Point( bw_dilate.cols/2 ,bw_dilate.rows/2 ), 255, &filledSize );
		//imshow("bw_dilate_filled", bw_dilate);

		/* floodFill not suppose to fill all the frame */
		if (filledSize.width * filledSize.height < detectionParameters.filledPart * (imborder.cols * imborder.rows))
		{
			tableFound = true;
		}

		/* Take pixels that are in the image inverse and in filled image */
		bitwise_and( bw_dilate, bw_inv, tableMask );
		//imshow("tableMask", tableMask);

		/* Remove small objects from binary image */
		removeSmallBlobs( tableMask, detectionParameters.minAreaTable );
		//imshow("Remove small objects", tableMask);

		/* Dilate image to fill holes*/
		for (int i = 0; i < 1; i++)
		{
			dilate( tableMask, tableMask, detectionParameters.seTable );
		}
		//imshow("tableMask", tableMask);

		/* Close image */
		morphologyEx(tableMask,tableMask,MORPH_CLOSE,detectionParameters.seTable);
		//imshow("closed image", tableMask);

		/* Resize the image mask back to the full resolution */
		resize(tableMask, tableMask, Size(),  systemParameters.scale, systemParameters.scale);
		//imshow("resized back", tableMask);

		/* RGB to Gray scale and Multiply the frame with the mask */
		cvtColor( m_frame_RGB, m_frame_GRAY, CV_BGR2GRAY);
		bitwise_and( m_frame_GRAY, tableMask, m_frame_GRAY);

		// imshow("Table Detection", m_frame_GRAY);
		//waitKey(20);
		//waitKey(20);

	}
	catch(...)
	{
		cout << "failed in current frame: detectTable" << endl;
	}
	waitKey(50);
	return tableFound;
}


Stats Camera_i::regionStats(const vector<vector<Point> > contours, const int idx, const bool addRobots)
{
	// TODO initialize stats!!!
	Stats stats;

	// centroid      stats.centroid = calculateCentroid(contours, idx);
	// TODO check if NULL is appropriate value to return (NULL - set to (0.0, 0.0) )
	stats.centroid = Point2d(NULL);
	Moments m = moments(contours[idx]);

	if (m.m00 != 0.0)
	{
		double cx = m.m10 / m.m00;
        double cy = m.m01 / m.m00;
		stats.centroid = Point2d(cx, cy);		
	}
		
	// bounding box
	stats.bbox = boundingRect(contours[idx]);

	if (addRobots == true)
	{
		//area
		stats.area = contourArea(contours[idx]);

		//eccentricity = sqrt( 1 - (ma/MA)^2) --- ma= minor axis --- MA= major axis
		RotatedRect tempEllipse = fitEllipse(contours[idx]);

		float axisRatio = pow((tempEllipse.size.width / tempEllipse.size.height),2);
		stats.eccentricity = sqrt( 1 - axisRatio );
			
		// solidity = contour area / convex hull area
		vector<vector<Point> >hull( contours.size() );
		convexHull(contours[idx], hull[idx]);
		stats.solidity = contourArea(contours[idx]) / float(contourArea(hull[idx]));
	}
    return stats;
}


bool Camera_i::isRobot(const Stats stats)
{
	if(stats.eccentricity > detectionParameters.minEccentricity && stats.eccentricity < detectionParameters.maxEccentricity &&
		stats.solidity > detectionParameters.minSolidity && stats.solidity < detectionParameters.maxSolidity &&
		stats.area > detectionParameters.minAreaRobot)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void Camera_i::create_sBox(const Stats stats, Rect& sBox)
{
	// Create search boxes by enlarging the bounding boxes 
	int resize = (stats.bbox.width + stats.bbox.height) / 2;
	sBox.x = stats.bbox.x - resize;
	sBox.y = stats.bbox.y - resize;
	sBox.width =  stats.bbox.width + 2 * resize;
	sBox.height = stats.bbox.height + 2 * resize;
}


void Camera_i::drawDetection(const Stats stats, Rect sBox)
{
	// Draws Bounding and Search boxes
	circle(m_frame_RGB, stats.centroid, 1, RED, 2);	// centroid

	rectangle( m_frame_RGB, stats.bbox, YELLOW, 2);	// bounding box
	putText(m_frame_RGB, "bBox", Point(stats.bbox.x, stats.bbox.y-5), 1, 1, YELLOW);	

	rectangle( m_frame_RGB, sBox, MAGENTA, 2);	// search box
	putText(m_frame_RGB, "sBox", Point(sBox.x, sBox.y-5), 1, 1, MAGENTA);
}


void Camera_i::detectRobots( int addRobots)
{
	Mat bw_frame;
	Mat bw_contours = Mat::zeros( m_frame_GRAY.size(), CV_8UC1 );
	vector<Stats*> statsVector; 
	Stats stats;
	bool detectedRobot = false;
	Rect sBox;

	// TODO configure camera parameters to remove imadjust
	try
	{
		// threshold(m_frame_GRAY, bw_frame, detectionParameters.bwThreshold, 255, THRESH_BINARY_INV); /* Gray sclale to BW */
		bw_frame = m_frame_GRAY > 128;
		imshow("BW frame", bw_frame);
		waitKey(20);

		morphologyEx(bw_frame,bw_frame,MORPH_CLOSE,detectionParameters.seRobot);           /* Morphologically close image */
		imshow("closed image", bw_frame);
		waitKey(20);

		erode(bw_frame,bw_frame,detectionParameters.seErode);							   /* Erode image to remove possible connections between targets */
		imshow("Erode image", bw_frame);
		waitKey(20);

		/* New targets are added to the tracker (first frame or lost) 
			Use simple morphological target search */

		// Measure properties of image regions
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(bw_frame, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		// iterate through all the top-level contours

		for( int idx = 0; idx < contours.size(); idx++ )
		{
			
			//drawContours( bw_contours, contours, idx, RED, 2, 8, hierarchy, 0, Point() );
			imshow("Contours", bw_contours);
			// Measure properties of image regions
			try
			{
				if(contours[idx].size() < 5)
				{
					continue;
				}
				stats = regionStats( contours, idx, systemParameters.addRobots);	//TODO systemParameters.addRobots change to isRedetectionRequired
			}
			catch(...)
			{
				continue;
			}
			// First robot detection. New robots are added	
			if (systemParameters.addRobots == true)		//TODO change to isRedetectionRequired
			{
				detectedRobot = isRobot(stats);
				if (detectedRobot == true)
				{
					create_sBox(stats, sBox);	// Create search boxes by enlarging the bounding boxes  

					drawDetection(stats, sBox);	// Draws Centroids, Bounding and Search boxes

					// save centroid, bounding box and search box
					VisibleRobot* visbleRobot = new VisibleRobot(stats.centroid, stats.bbox, sBox); // todo: free robot's memory when removing it
					m_visibleRobots.push_back(visbleRobot);
				}
			}
			else    // Continue robot detection. No new robots were added.  systemParameters.addRobots == false
			{      // Search for the targets in the area it was previously seen
				
				create_sBox(stats, sBox);		// Create search boxes by enlarging the bounding boxes  
				
				// Search what centroid is located inside each search box
				// TODO: Search what centroid is located inside each search box 
				bool isContained = sBox.contains(stats.centroid);
			}
		
		}
		imshow("Draws Bounding and Search boxes", m_frame_RGB);
		
		waitKey(20);
	
		
	}
	catch(...)
	{
		cout << "failed in current frame: detectRobots" << endl;
	}


	waitKey(20);
	
}