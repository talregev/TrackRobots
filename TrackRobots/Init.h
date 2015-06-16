#ifndef INIT_H_
#define INIT_H_

#include "UserConfig.h"
#include "GlobalStructsDef.h"

#include <cv.h>
#include <highgui.h>
#include "Camera_i.h"


//#include <uEye.h>

using namespace cv;

bool system_init(Camera_i& camera);

#endif INIT_H_