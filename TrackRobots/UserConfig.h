#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_

/*****************************************************************************
*
*	UserConfig module is responsible for storing 
*	all the initial values for system setup.
*
******************************************************************************/

/* Capture Video Parameters */
#define CAMERA				0
#define AVI_FILE            1

#define VIDEO_INPUT			AVI_FILE /* CAMERA */ 		 /* in case of reading from file comment this line! */

/*System Parameters*/
#define ADD_ROBOTS			true	/* Flag to add robots to the tracker */
#define IM_SCALE			2		/* Resize images (imsize/scale = int) */


/* Detection Parameters */

/* HSV Parameters - from AVI file */
#define H_MIN_VALUE_FILE			165
#define S_MIN_VALUE_FILE			110
#define	V_MIN_VALUE_FILE			160
#define H_MAX_VALUE_FILE			200
#define	S_MAX_VALUE_FILE			200
#define	V_MAX_VALUE_FILE			190
#define CONTRAST_VALUE_FILE			2.1		/* contrast coeficient */
#define BRIGHTNESS_VALUE_FILE		25		/* brightness coeficient */


///* HSV Parameters - from camera input*/
//#define H_MIN_VALUE_CAMERA			150
//#define S_MIN_VALUE_CAMERA			50
//#define	V_MIN_VALUE_CAMERA			150
//#define H_MAX_VALUE_CAMERA			190
//#define	S_MAX_VALUE_CAMERA			165
//#define	V_MAX_VALUE_CAMERA			190
//#define CONTRAST_VALUE_CAMERA		5.5/* contrast coeficient */
//#define BRIGHTNESS_VALUE_CAMERA		70	/* brightness coeficient */

///* HSV Parameters - from camera input*/
#define H_MIN_VALUE_CAMERA			145
#define S_MIN_VALUE_CAMERA			51
#define	V_MIN_VALUE_CAMERA			110
#define H_MAX_VALUE_CAMERA			180
#define	S_MAX_VALUE_CAMERA			160
#define	V_MAX_VALUE_CAMERA			160
#define CONTRAST_VALUE_CAMERA		5.0/* contrast coeficient */
#define BRIGHTNESS_VALUE_CAMERA		65	/* brightness coeficient */


/* TABLE */
#define TABLE_MORPH_ELEMENT_SIZE	    25		 /* Create morphological structuring element for table detection */
#define TABLE_PART_FILLED				0.9      /* part of the filled table area in the image - as found in floodFill function
													for veriffing the fill result. floodFill not suppose to fill all the frame  */
#define MIN_TABLE_BLOB					500      /* Minimum BLOB area in Table detection(pixels) */

/* ROBOT */
#define BW_THRESHOLD                    40      /* Black and white transformation threshold */
#define MIN_ECCENTRICITY                0.73     /* Minimum Object Eccentricity threshold */
#define MAX_ECCENTRICITY                1        /* Maximum Object Eccentricity threshold */
#define MIN_SOLIDITY                    0.7      /* Minimum Object Solidity threshold */
#define MAX_SOLIDITY                    1        /* Maximum Object Solidity threshold */
#define MIN_ROBOT_BLOB                  15       /* Minimum BLOB area in Robot detection(pixels) */
#define ROBOT_MORPH_ELEMENT_SIZE	    8		 /* Create morphological structuring element for Robot detection */
#define ROBOT_MORPH_ERODE_SIZE	        2		 /* Create morphological erode element for Robot detection */


/* Display Parameters */
#define YELLOW					Scalar( 0, 255, 255 )		/* Yellow color   */
#define MAGENTA					Scalar( 255, 0, 255 )		/* Magenta color  */
#define RED					    Scalar(  0,  0, 255 )		/* Red color      */



#endif /* USER_CONFIG_H_ */