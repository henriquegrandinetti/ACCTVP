//  Plane Projection
//  TopView.h
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifndef __UoB_Project__TopView__
#define __UoB_Project__TopView__

#include <stdio.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

typedef struct mouseDataCrop{
    Point lastPoint;
    vector<Vec2f> rec;
}mouseDataCrop;

class TopView{
private:
    Mat image;
    Mat M, Mi;
    float f;
    Vec3f Fu, Fv;
    Vec3f u, v, w;
    Vec3f O;
    float sf; //scale factor
    mouseDataCrop *mouseData;
    Mat transformationMat;
public:
    Mat topImage;
    
private:
    Vec2f verticalAxis();
    void ComputeUVW();
    void ComputeM();
    Vec3f convertToCamCoord(Vec3f A);
    Vec3f convertToWorldCoord(Vec3f A);
    Point IPProjection(Vec3f P);
    
public:
    TopView(Mat img, Vec2f vp1, Vec2f vp2, mouseDataCrop *mouse);
    void drawAxis(Mat output, Point p);
    void setOrigin(Vec2f p);
    void setScaleFactor(Vec2f a, Vec2f b, float dist);
    Vec2f toGroundPlaneCoord(Vec2f a);
    void generateTopImage();
    void cropTopView();
    vector<Vec2f> toTopViewCoordinates(vector<Vec2f> a);

};

#endif /* defined(__UoB_Project__TopView__) */