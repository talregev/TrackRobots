#include "Init.h"

#include <uEye.h>
#include <stdio.h>
#include <stddef.h>
#include <cv.h>


void hsvParametersInit();
void detectionParametersInit();
void systemParametersInit();
void kalmanFilterInit();


bool system_init(Camera_i& camera)
{
	bool res = false;
	systemParametersInit();
	hsvParametersInit();	
	detectionParametersInit();

	kalmanFilterInit();
	res = camera.cameraInit();

	return res;
}

void systemParametersInit()
{
	systemParameters.addRobots      = ADD_ROBOTS;			/* Flag to add robots to the tracker */
	systemParameters.scale			= IM_SCALE;				/* Resize images (imsize/scale = int) */
}


void hsvParametersInit()
{
	#if (VIDEO_INPUT == AVI_FILE)
	{
		hsvParameters.hsvColors[H_MIN] = H_MIN_VALUE_FILE;
		hsvParameters.hsvColors[S_MIN] = S_MIN_VALUE_FILE;
		hsvParameters.hsvColors[V_MIN] = V_MIN_VALUE_FILE;
		hsvParameters.hsvColors[H_MAX] = H_MAX_VALUE_FILE;
		hsvParameters.hsvColors[S_MAX] = S_MAX_VALUE_FILE;
		hsvParameters.hsvColors[V_MAX] = V_MAX_VALUE_FILE;

		hsvParameters.contrastValue	   = CONTRAST_VALUE_FILE;			 /* contrast coeficient */
		hsvParameters.brightnessValue  = BRIGHTNESS_VALUE_FILE;			 /* brightness coeficient */
	}
	#else
	{
		hsvParameters.hsvColors[H_MIN] = H_MIN_VALUE_CAMERA;
		hsvParameters.hsvColors[S_MIN] = S_MIN_VALUE_CAMERA;
		hsvParameters.hsvColors[V_MIN] = V_MIN_VALUE_CAMERA;
		hsvParameters.hsvColors[H_MAX] = H_MAX_VALUE_CAMERA;
		hsvParameters.hsvColors[S_MAX] = S_MAX_VALUE_CAMERA;
		hsvParameters.hsvColors[V_MAX] = V_MAX_VALUE_CAMERA;

		hsvParameters.contrastValue	   = CONTRAST_VALUE_CAMERA;			 /* contrast coeficient */
		hsvParameters.brightnessValue  = BRIGHTNESS_VALUE_CAMERA;			 /* brightness coeficient */
	}
	#endif
}

void detectionParametersInit()
{
	/* TABLE DETECTION */

	Size ksize_1 = Size(TABLE_MORPH_ELEMENT_SIZE, TABLE_MORPH_ELEMENT_SIZE);  
	detectionParameters.seTable = getStructuringElement(MORPH_ELLIPSE , ksize_1); /* Create morphological structuring element for Table detection */

	detectionParameters.filledPart = TABLE_PART_FILLED;                         /* part of the filled table area in the image */
	detectionParameters.minAreaTable = MIN_TABLE_BLOB;		                    /* Minimum BLOB area in table detection(pixels) */

	/* ROBOT DETECTION */

	detectionParameters.bwThreshold = BW_THRESHOLD;		                        /* Black and white transformation threshold */
	detectionParameters.minEccentricity = MIN_ECCENTRICITY;                     /* Minimum Object Eccentricity threshold */
	detectionParameters.maxEccentricity = MAX_ECCENTRICITY;                     /* Maximum Object Eccentricity threshold */
	detectionParameters.minSolidity = MIN_SOLIDITY;		                        /* Minimum Object Solidity threshold */
	detectionParameters.maxSolidity = MAX_SOLIDITY;		                        /* Maximum Object Solidity threshold */
    detectionParameters.minAreaRobot = MIN_ROBOT_BLOB;		                    /* Minimum BLOB area in robot detectio (pixels) */

	Size ksize_2 = Size(ROBOT_MORPH_ELEMENT_SIZE, ROBOT_MORPH_ELEMENT_SIZE);  
	detectionParameters.seRobot = getStructuringElement(MORPH_ELLIPSE , ksize_2); /* Create morphological structuring element for Robot detection */
	Size ksize_3 = Size(ROBOT_MORPH_ERODE_SIZE, ROBOT_MORPH_ERODE_SIZE);  
	detectionParameters.seErode = getStructuringElement(MORPH_ELLIPSE , ksize_3); /* Create morphological erode element for Robot detection */
}



void kalmanFilterInit()
{

}

