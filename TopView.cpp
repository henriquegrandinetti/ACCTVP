//  Plane Projection
//  TopView.cpp
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#include "TopView.h"
#include "geometry.h"

#include <iostream>

TopView::TopView(Mat img, Point2f vp1, Point2f vp2, mouseDataCrop *mouse){
    image = Mat(img).clone();
    ref = Point2f(image.cols/2, image.rows/2);
    mouseData = mouse;
    transformationMat = Mat(3,3, CV_8UC1);
    
    //vanishing points in 2D
    Point2f fu(vp1.x - ref.x, vp1.y - ref.y);
    Point2f fv(vp2.x - ref.x, vp2.y - ref.y);
    
    Point2f P(0,0);
    
    float tant = (float)(fv.y - fu.y)/(fv.x - fu.x);
    float t = atan(tant) + CV_PI/2;
    float r = linePointDist(fv, fu, P, true);
    
    Vec2f Puv(r * cos(t), -(r * sin(t)));
    
    float OPuv = sqrt(pointDistance(fv, Puv) * pointDistance(Puv, fu));
    
    f = sqrt(OPuv * OPuv - pow(pointDistance(P, Puv), 2));
    Fu = Vec3f(fu.x, fu.y, f);
    Fv = Vec3f(fv.x, fv.y, f);
    
    //initialize M
    M  = Mat(3, 3, CV_32F);
    Mi = Mat(3, 3, CV_32F);
    
    sf = 1.0;
    O = Point3f(0.0,0.0,0.0);
    
    //must follow this order
    ComputeUVW();
    ComputeM();
}

void TopView::ComputeUVW(){
    Vec3f O(0, 0, 0);
    
    Vec3f Fu3(Fu.x, Fu.y, f);
    Vec3f OFu3 = Fu3 - O;
    float s1 = length(OFu3);
    
    Vec3f Fv3(Fv.x, Fv.y, f);
    Vec3f OFv3 = Fv3 - O;
    float s2 = length(OFv3);
    
    u = Vec3f(Fu.x/s1, Fu.y/s1, f/s1);
    v = Vec3f(Fv.x/s2, Fv.y/s2, f/s2);
    w = cross_product(u, v);
    
    //swap w if defined down
    if (w[1] > 0) {
        Vec3f temp(v);
        v = Vec3f(u);
        u = Vec3f(temp);
        w *= -1;
    }
}

void TopView::ComputeM(){
    
    M.at<float>(Point(0,0)) = u[0];
    M.at<float>(Point(0,1)) = v[0];
    M.at<float>(Point(0,2)) = w[0];
    M.at<float>(Point(1,0)) = u[1];
    M.at<float>(Point(1,1)) = v[1];
    M.at<float>(Point(1,2)) = w[1];
    M.at<float>(Point(2,0)) = u[2];
    M.at<float>(Point(2,1)) = v[2];
    M.at<float>(Point(2,2)) = w[2];
    
    Mi = M.inv();
}

void TopView::drawAxis(Mat output, Point p){
    
    Point3f Pw = convertToWorldCoord(Point3f(p.x, p.y, f));
    
    Point3f uAxis = Point3f(1,0,0) * 100 + Pw;
    Point3f vAxis = Point3f(0,1,0) * 100 + Pw;
    Point3f wAxis = Point3f(0,0,1) * 100 + Pw;
    
    uAxis = convertToCamCoord(uAxis);
    vAxis = convertToCamCoord(vAxis);
    wAxis = convertToCamCoord(wAxis);
            
    arrowedLine(output, p + ref, IPProjection(uAxis), Scalar(0,0,255));
    arrowedLine(output, p + ref, IPProjection(vAxis), Scalar(0,255,0));
    arrowedLine(output, p + ref, IPProjection(wAxis), Scalar(255,0,0));
}

Point3f TopView::convertToCamCoord(Point3f A){
    Point3f result;
    result.x = M.at<float>(Point(0, 0)) * A.x + M.at<float>(Point(0, 1)) * A.y + M.at<float>(Point(0, 2)) * A.z;
    result.y = M.at<float>(Point(1, 0)) * A.x + M.at<float>(Point(1, 1)) * A.y + M.at<float>(Point(1, 2)) * A.z;
    result.z = M.at<float>(Point(2, 0)) * A.x + M.at<float>(Point(2, 1)) * A.y + M.at<float>(Point(2, 2)) * A.z;
    
    return result;
}

Point3f TopView::convertToWorldCoord(Point3f A){
    Point3f result;
    result.x = Mi.at<float>(Point(0, 0)) * A.x + Mi.at<float>(Point(0, 1)) * A.y + Mi.at<float>(Point(0, 2)) * A.z;
    result.y = Mi.at<float>(Point(1, 0)) * A.x + Mi.at<float>(Point(1, 1)) * A.y + Mi.at<float>(Point(1, 2)) * A.z;
    result.z = Mi.at<float>(Point(2, 0)) * A.x + Mi.at<float>(Point(2, 1)) * A.y + Mi.at<float>(Point(2, 2)) * A.z;
    
    return result;
}

//projection of the point in the image plane
Point TopView::IPProjection(Point3f P){
    normalize_vec(P);
    Point3f P2 = f/P.z * P;
    
    return Point(P2.x, P2.y) + ref;
}

void TopView::generateTopImage(){
    
    //assume center of image is on the ground plane
    Point3f P = convertToWorldCoord(Point3f(0, 0, f));
    
    int xDirection = w[0]/abs(w[0]);
    int yDirection = w[1]/abs(w[1]);
    
    float a = (Fu.y - Fv.y)/(Fu.x - Fv.x);
    float b = -a * Fu.x + Fu.y;
    
    vector<Point> points;
    
    Point A(-ref.x, a * -ref.x + b - yDirection * 20);
    Point B(ref.x, a * ref.x + b - yDirection * 20);
    Point C((-ref.y - b)/a - xDirection * 20, -ref.y);
    Point D((ref.y - b)/a - xDirection * 20, ref.y);
    
    //check if vanishing line crosses borders
    if (A.y >= -ref.y && A.y <= image.rows - ref.y)
        points.push_back(A);
    if (B.y >= -ref.y && B.y <= image.rows - ref.y)
        points.push_back(B);
    if (C.x >= -ref.x && C.x <= image.cols - ref.x)
        points.push_back(C);
    if (D.x >= -ref.x && D.x <= image.cols - ref.x)
        points.push_back(D);
    
    if (points.size() <= 1) {
        points.clear();
        points.push_back(Point(-ref.x, -ref.y));
        points.push_back(Point(ref.x, -ref.y));
        points.push_back(Point(-ref.x, ref.y));
        points.push_back(Point(ref.x, ref.y));
    }
    else{
        for (int i = 0; i < 2; i++) {
            if(points[i].y == -ref.y)
                points.push_back(Point(-xDirection * ref.x, -ref.y));
            else if(points[i].y == ref.y)
                points.push_back(Point(-xDirection * ref.x, ref.y));
            else if(points[i].x == -ref.x)
                points.push_back(Point(-ref.x, -yDirection * ref.y));
            else if(points[i].x == ref.x)
                points.push_back(Point(ref.x, -yDirection * ref.y));
        }
    }
    
    Vec3f temp[4];
    Point2f source_points[4];
    Point2f dest_points[4];
    
    for (int i = 0; i < 4; i++) {
        temp[i] = vecPlaneInter(P, convertToWorldCoord(Vec3f(points[i].x, points[i].y, f)));
        source_points[i] = Point2f(points[i].x + ref.x, points[i].y + ref.y);
        dest_points[i] = Point2f(temp[i][0], temp[i][1]);
    }
    
    
    //fit points into image rectangle
    topImage = Mat(image.rows, image.cols, image.type());
    
    fitQuadRec(dest_points, dest_points, Size(topImage.cols, topImage.rows));
    
    Mat transform_matrix(3,3, CV_8UC1);
    
    transform_matrix = getPerspectiveTransform(source_points, dest_points);
    
    float height = topImage.rows;
    //if image is croped
    if (mouseData->rec.size() > 1) {
        mouseData->rec.push_back(Point2f(mouseData->rec[1].x, mouseData->rec[0].y));
        mouseData->rec.push_back(Point2f(mouseData->rec[0].x, mouseData->rec[1].y));
        
        height = (float)(mouseData->rec[1].y - mouseData->rec[0].y)/(mouseData->rec[1].x - mouseData->rec[0].x) * topImage.cols;
        
        vector<Point2f> transformed;
        perspectiveTransform(mouseData->rec, transformed, transform_matrix.inv());
        
        source_points[0] = transformed[0];
        source_points[1] = transformed[2];
        source_points[2] = transformed[3];
        source_points[3] = transformed[1];
        
        dest_points[0] = Point(0,0);
        dest_points[1] = Point(topImage.cols, 0);
        dest_points[2] = Point(0, height);
        dest_points[3] = Point(topImage.cols, height);
        
        transform_matrix = getPerspectiveTransform(source_points, dest_points);
    }
    
    transformationMat = transform_matrix.clone();
    
    warpPerspective(image, topImage, transform_matrix, Size(topImage.cols, (int)height));
}


void mouseCrop(int event, int x, int y, int flags, void* userdata){
    mouseDataCrop *data = (mouseDataCrop *) userdata;
    if  (event == EVENT_LBUTTONDOWN ){
        if (data->rec.size() < 2)
            data->rec.push_back(Point(x,y));
    }
    else if (event == EVENT_MOUSEMOVE ){
        data->lastPoint = Point(x,y);
    }
    userdata = (void *) &data;
}

void TopView::cropTopView(){
    namedWindow( mouseData->windowName, WINDOW_AUTOSIZE );
    setMouseCallback(mouseData->windowName, mouseCrop, (void *)mouseData);
    
    if (mouseData->rec.size() == 1) {
        line(topImage, Point(mouseData->rec[0].x,mouseData->rec[0].y), Point(mouseData->lastPoint.x, mouseData->rec[0].y), Scalar(0,0,255));
        line(topImage, Point(mouseData->lastPoint.x, mouseData->lastPoint.y), Point(mouseData->lastPoint.x, mouseData->rec[0].y), Scalar(0,0,255));
        line(topImage, Point(mouseData->lastPoint.x, mouseData->lastPoint.y), Point(mouseData->rec[0].x,mouseData->lastPoint.y), Scalar(0,0,255));
        line(topImage, Point(mouseData->rec[0].x,mouseData->rec[0].y), Point(mouseData->rec[0].x,mouseData->lastPoint.y), Scalar(0,0,255));
    }
}

void TopView::setOrigin(Point p){
    Point3f P = convertToWorldCoord(Vec3f(0, 0, f));
    O = vecPlaneInter(P, convertToWorldCoord(Vec3f(p.x - ref.x, p.y - ref.y, f)));
}

void TopView::setScaleFactor(Point a, Point b, float dist){
    Point3f P = convertToWorldCoord(Vec3f(0, 0, f));
    Point3f A = vecPlaneInter(P, convertToWorldCoord(Point3f(a.x - ref.x, a.y - ref.y, f)));
    Point3f B = vecPlaneInter(P, convertToWorldCoord(Point3f(b.x - ref.x, b.y - ref.y, f)));
    float d = pointDistance(A, B);
    
    sf = d/dist;
}

Point2f TopView::toGroundPlaneCoord(Point a){
    Point3f P = convertToWorldCoord(Point3f(0, 0, f));
    Point3f A = vecPlaneInter(P, convertToWorldCoord(Vec3f(a.x - ref.x, a.y - ref.y, f)));
    
    //translate and scale
    A -= O;
    A.x /= sf; A.y /= sf; A.z /= sf;
    
    return Point2f(A.x, A.y);
}

vector<Vec2f> TopView::toTopViewCoordinates(vector<Vec2f> a){
    vector<Vec2f> result;
    perspectiveTransform(a, result, transformationMat);
    
    return result;
}