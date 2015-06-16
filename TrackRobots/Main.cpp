#include <cv.h>
#include <highgui.h>
#include <iostream>

// TODO - change to relative path
#include "C:\Program Files\IDS\uEye\Develop\include\uEye.h"
#include "Init.h"
#include "Camera_i.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{	
	bool isSuccessfulInit = false;
	int nframes = 0;
	bool endvideoflag = false;
	bool redetectionrequired = true;
	bool tableFound = false;
	Camera_i camera;  // TODO - rename
	Mat imBorder;
	Mat frameHSV;


	/* initialize ALL system global structs. Using UserConfig values when applicable. */
	isSuccessfulInit = system_init(camera);

	if(isSuccessfulInit == true)
	{
		while(endvideoflag == false)
		{
			tableFound = false;
			endvideoflag = camera.captureFrame();
			
			redetectionrequired = camera.isReDetectionRequired();
			if (redetectionrequired == true)
			{
				/* detectborders - hsv transform to detect red collor on table border*/
				imBorder = camera.detectColor(frameHSV);
        
				/* table detection - retry till detection successeded  */
				int attemptsCount = 1;
				while((tableFound == false) && (attemptsCount < 5))
				{
					tableFound = camera.detectTable(imBorder, attemptsCount);
					attemptsCount++;
				}

				if (attemptsCount > 2)
				{
					cout << "attemptsCount is: " << attemptsCount << endl;
				}
				 //redetectionrequired = false;
			}
			
			// todo - check opencv signature: frame = rgb2gray(frame);    %convert from rgb to gray scale

			/* ROBOT DETECTION*/
			camera.detectRobots(systemParameters.addRobots);
		}
	}
	else
	{
		// initialization failed - error code reported as stored in errno
		return 1;
	}
	return 0;
}