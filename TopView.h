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
    vector<Point2f> rec;
    string windowName;
}mouseDataCrop;

class TopView{
private:
    Mat image;
    Mat M, Mi;
    float f;
    Point3f Fu, Fv;
    Vec3f u, v, w;
    Point3f O;
    Point ref;
    float sf; //scale factor
    mouseDataCrop *mouseData;
    Mat transformationMat;
public:
    Mat topImage;
    
private:
    Vec2f verticalAxis();
    void ComputeUVW();
    void ComputeM();
    Point3f convertToCamCoord(Point3f A);
    Point3f convertToWorldCoord(Point3f A);
    Point IPProjection(Point3f P);
    
public:
    TopView(Mat img, Point2f vp1, Point2f vp2, mouseDataCrop *mouse);
    void drawAxis(Mat output, Point p);
    void setOrigin(Point p);
    void setScaleFactor(Point a, Point b, float dist);
    Point2f toGroundPlaneCoord(Point a);
    void generateTopImage();
    void cropTopView();
    vector<Vec2f> toTopViewCoordinates(vector<Vec2f> a);

};

#endif