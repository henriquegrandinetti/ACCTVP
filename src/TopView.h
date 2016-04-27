//  Plane Projection
//  TopView.h
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifndef __ACCTVP__TopView__
#define __ACCTVP__TopView__

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
public:
    Mat topImage;
    
    TopView(Mat img, Point2f vp1, Point2f vp2, mouseDataCrop *mouse);
    void drawAxis(Mat output, Point p);
    void setOrigin(Point p);
    void setScaleFactor(Point a, Point b, float dist);
    Point2f toGroundPlaneCoord(Point a);
    void generateTopImage();
    void cropTopView();
    vector<Point2f> toTopViewCoordinates(vector<Point2f> a);
    
private:
    Mat image;
    Mat M, Mi;
    Point3f Fu, Fv;
    Point3f O;
    Point ref;
    Vec3f u, v, w;
    float f;
    float sf; //scale factor
    mouseDataCrop *mouseData;
    Mat transformationMat;
    
    Vec2f verticalAxis();
    void ComputeUVW();
    void ComputeM();
    Point3f convertToCamCoord(Point3f A);
    Point3f convertToWorldCoord(Point3f A);
    Point IPProjection(Point3f P);
    

};

#endif