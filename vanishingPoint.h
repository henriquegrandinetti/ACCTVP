//  Plane Projection
//  vanishingPoint.h
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifndef __UoB_Project__vanishingPoint__
#define __UoB_Project__vanishingPoint__

#include <stdio.h>
#include "opencv2/core/core.hpp"

typedef struct mouseDataVP{
    bool clicked;
    bool uDone;
    Point a, b;
    vector<Vec4f> fumanual, fvmanual;
    Mat image;
} mouseDataVP;

void callbackFunc(int event, int x, int y, int flags, void* userdata);

Vec4f manualCalibration(mouseDataVP *data);

Vec4f automaticCalibration(MSAC &msac, int numVps, cv::Mat &imgGRAY, cv::Mat &outputImg, int houghThreshold);

bool validVPS(Vec4f vps);

#endif /* defined(__UoB_Project__vanishingPoint__) */