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

TopView::TopView(Mat img, Vec2f vp1, Vec2f vp2, mouseDataCrop *mouse){
    image = Mat(img);
    Vec2f ref(image.cols/2, image.rows/2);
    mouseData = mouse;
    
    //vanishing points in 2D
    Vec2f fu(vp1[0] - ref[0], vp1[1] - ref[1]);
    Vec2f fv(vp2[0] - ref[0], vp2[1] - ref[1]);
    
    Vec2f P(0,0);
    
    float tant = (float)(fv[1] - fu[1])/(fv[0] - fu[0]);
    float t = atan(tant) + CV_PI/2;
    float r = linePointDist(fv, fu, P, true);
    
    Vec2f Puv(r * cos(t), -(r * sin(t)));
    
    float OPuv = sqrt(pointDistance(fv, Puv) * pointDistance(Puv, fu));
    
    f = sqrt(OPuv * OPuv - pow(pointDistance(P, Puv), 2));
    Fu = Vec3f(fu[0], fu[1], f);
    Fv = Vec3f(fv[0], fv[1], f);
    
    //initialize M
    M  = Mat(3, 3, CV_32F);
    Mi = Mat(3, 3, CV_32F);
    
    sf = 1;
    O = Vec3f(0,0,0);
    
    ComputeUVW();
    ComputeM();
}

void TopView::ComputeUVW(){
    Vec3f O(0, 0, 0);
    
    Vec3f Fu3(Fu[0], Fu[1], f);
    Vec3f OFu3 = Fu3 - O;
    float s1 = length(OFu3);
    
    Vec3f Fv3(Fv[0], Fv[1], f);
    Vec3f OFv3 = Fv3 - O;
    float s2 = length(OFv3);
    
    u = Vec3f(Fu[0]/s1, Fu[1]/s1, f/s1);
    v = Vec3f(Fv[0]/s2, Fv[1]/s2, f/s2);
    w = cross_product(u, v);
    
    //swap w if defined down
    /*if (w[1] > 0) {
     Vec3f temp(v);
     v = Vec3f(u);
     u = Vec3f(temp);
     w *= -1;
     }*/
}

void TopView::ComputeM(){
    
    //float denom1 = sqrt(Fu[0] * Fu[0] + Fu[1] * Fu[1] + f * f);
    //float denom2 = sqrt(Fv[0] * Fv[0] + Fv[1] * Fv[1] + f * f);
    
    /*float denom1 = sqrt(Fu[0] * Fu[0] + Fu[1] * Fu[1] + f * f);
     float denom2 = sqrt(Fv[0] * Fv[0] + Fv[1] * Fv[1] + f * f);
     
     M.at<float>(Point(0,0)) = Fu[0]/denom1;
     M.at<float>(Point(0,1)) = Fv[0]/denom2;
     M.at<float>(Point(0,2)) = w[0];
     M.at<float>(Point(1,0)) = Fu[1]/denom1;
     M.at<float>(Point(1,1)) = Fv[1]/denom2;
     M.at<float>(Point(1,2)) = w[1];
     M.at<float>(Point(2,0)) = f/denom1;
     M.at<float>(Point(2,1)) = f/denom2;
     M.at<float>(Point(2,2)) = w[2];*/
    
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
    
    Vec3f Pw = convertToWorldCoord(Vec3f(p.x, p.y, f));
    
    Vec3f uAxis = Vec3f(1,0,0) * 5000 + Pw;
    Vec3f vAxis = Vec3f(0,1,0) * 5000 + Pw;
    Vec3f wAxis = Vec3f(0,0,1) * 5000 + Pw;

    uAxis = convertToCamCoord(uAxis);
    vAxis = convertToCamCoord(vAxis);
    wAxis = convertToCamCoord(wAxis);
    
    Point ref(output.cols/2, output.rows/2);
    
    line(output, p + ref, IPProjection(uAxis), Scalar(0,0,255));
    line(output, p + ref, IPProjection(vAxis), Scalar(0,255,0));
    line(output, p + ref, IPProjection(wAxis), Scalar(255,0,0));
}

Vec3f TopView::convertToCamCoord(Vec3f A){
    Vec3f result;
    result[0] = M.at<float>(Point(0, 0)) * A[0] + M.at<float>(Point(0, 1)) * A[1] + M.at<float>(Point(0, 2)) * A[2];
    result[1] = M.at<float>(Point(1, 0)) * A[0] + M.at<float>(Point(1, 1)) * A[1] + M.at<float>(Point(1, 2)) * A[2];
    result[2] = M.at<float>(Point(2, 0)) * A[0] + M.at<float>(Point(2, 1)) * A[1] + M.at<float>(Point(2, 2)) * A[2];
    
    return result;
}

Vec3f TopView::convertToWorldCoord(Vec3f A){
    Vec3f result;
    result[0] = Mi.at<float>(Point(0, 0)) * A[0] + Mi.at<float>(Point(0, 1)) * A[1] + Mi.at<float>(Point(0, 2)) * A[2];
    result[1] = Mi.at<float>(Point(1, 0)) * A[0] + Mi.at<float>(Point(1, 1)) * A[1] + Mi.at<float>(Point(1, 2)) * A[2];
    result[2] = Mi.at<float>(Point(2, 0)) * A[0] + Mi.at<float>(Point(2, 1)) * A[1] + Mi.at<float>(Point(2, 2)) * A[2];
    
    return result;
}

//projection of the point in the image plane
Point TopView::IPProjection(Vec3f P){
    normalize_vec(P);
    Vec3f x = f/P[2] * P;
    
    return Point(x[0] + image.cols/2, x[1] + image.rows/2);
}

void TopView::generateTopImage(){
    Vec2f ref(image.cols/2, image.rows/2);
    
    //assume center of image is on the ground plane
    Vec3f P = convertToWorldCoord(Vec3f(0, 0, f));
    
    ////////////////
    
    int xDirection = w[0]/abs(w[0]);
    int yDirection = w[1]/abs(w[1]);
    
    vector<Point> points;
    
    float a = (Fu[1] - Fv[1])/(Fu[0] - Fv[0]);
    float b = -a * Fu[0] + Fu[1];
    
    Point A(-ref[0], a * -ref[0] + b + 20);
    Point B(ref[0], a * ref[0] + b + 20);
    Point C((-ref[1] - b)/a - 20, -ref[1]);
    Point D((ref[1] - b)/a - 20, ref[1]);
    
    if (A.y >= -ref[1] && A.y <= image.rows - ref[1])
        points.push_back(A);
    if (B.y >= -ref[1] && B.y <= image.rows - ref[1])
        points.push_back(B);
    if (C.x >= -ref[0] && C.x <= image.cols - ref[0])
        points.push_back(C);
    if (D.x >= -ref[0] && D.x <= image.cols - ref[0])
        points.push_back(D);
    
    if (points.size() <= 1) {
        points.clear();
        points.push_back(Point(-ref[0], -ref[1]));
        points.push_back(Point(ref[0], -ref[1]));
        points.push_back(Point(-ref[0], ref[1]));
        points.push_back(Point(ref[0], ref[1]));
    }
    else{
        for (int i = 0; i < 2; i++) {
            if(points[i].y == -ref[1])
                points.push_back(Point(xDirection * ref[0], -ref[1]));
            else if(points[i].y == ref[1])
                points.push_back(Point(xDirection * ref[0], ref[1]));
            else if(points[i].x == -ref[0])
                points.push_back(Point(-ref[0], yDirection * ref[1]));
            else if(points[i].x == ref[0])
                points.push_back(Point(ref[0], yDirection * ref[1]));
        }
    }
    
    
    ////////////////
    /*
     Point2f tls, trs, bls, brs;
     
     //find 3D coord of the corners on the plane
     Vec3f tl, tr;
     
     if (A.y < -ref[1] || A.y > ref[1]){
     tl = vecPlaneInter(P, convertToWorldCoord(Vec3f(-ref[0], -ref[1], f)));
     tls = Point2f(0,0);
     }
     else{
     tl = vecPlaneInter(P, convertToWorldCoord(Vec3f(A.x, A.y, f)));
     tls = Point2f(A.x + ref[0],A.y + ref[1]);
     }
     if (B.y < -ref[1] || B.y > ref[1]){
     tr = vecPlaneInter(P, convertToWorldCoord(Vec3f(ref[0], -ref[1], f)));
     trs = Point2f(image.cols,0);
     }
     else{
     tr = vecPlaneInter(P, convertToWorldCoord(Vec3f(B.x, B.y, f)));
     trs = Point2f(B.x + ref[0], B.y + ref[1]);
     }
     
     Vec3f bl = vecPlaneInter(P, convertToWorldCoord(Vec3f(-ref[0], ref[1], f)));
     Vec3f br = vecPlaneInter(P, convertToWorldCoord(Vec3f(ref[0], ref[1], f)));
     
     cv::Point2f source_points[4];
     cv::Point2f dest_points[4];
     
     //map points from perspective to top view
     source_points[0] = Point2f(0);
     source_points[1] = Point(image.cols, 0);
     source_points[2] = Point2f(0,image.rows);
     source_points[3] = Point2f(image.cols,image.rows);
     
     dest_points[0] = Point2f(tl[0], tl[1]);
     dest_points[1] = Point2f(tr[0], tr[1]);
     dest_points[2] = Point2f(bl[0], bl[1]);
     dest_points[3] = Point2f(br[0], br[1]);*/
    
    Vec3f bla[4];
    cv::Point2f source_points[4];
    cv::Point2f dest_points[4];
    
    for (int i = 0; i < 4; i++) {
        bla[i] = vecPlaneInter(P, convertToWorldCoord(Vec3f(points[i].x, points[i].y, f)));
        source_points[i] = Point2f(points[i].x + ref[0], points[i].y + ref[1]);
        dest_points[i] = Point2f(bla[i][0], bla[i][1]);
    }
    
    //fit points into image rectangle
    topImage = Mat(image.rows, image.cols, image.type());
    
    fitQuadRec(dest_points, dest_points, Size(topImage.cols, topImage.rows));
    
    Mat transform_matrix(3,3, CV_8UC1);
    
    transform_matrix = getPerspectiveTransform(source_points, dest_points);
    
    float height = topImage.rows;
    if (mouseData->rec.size() > 1) {
        mouseData->rec.push_back(Vec2f(mouseData->rec[1][0], mouseData->rec[0][1]));
        mouseData->rec.push_back(Vec2f(mouseData->rec[0][0], mouseData->rec[1][1]));
        
        height = (float)(mouseData->rec[1][1]-mouseData->rec[0][1])/(mouseData->rec[1][0]-mouseData->rec[0][0]) * topImage.cols;
        
        vector<Vec2f> transformed;
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
    
    warpPerspective(image, topImage, transform_matrix, Size(topImage.cols, (int)height));
}


void mouseCrop(int event, int x, int y, int flags, void* userdata){
    mouseDataCrop *data = (mouseDataCrop *) userdata;
    if  (event == EVENT_LBUTTONDOWN ){
        if (data->rec.size() < 2)
            data->rec.push_back(Vec2f(x,y));
    }
    else if (event == EVENT_MOUSEMOVE ){
        data->lastPoint = Point(x,y);
    }
    userdata = (void *) &data;
}

void TopView::cropTopView(){
    namedWindow( "Top View", WINDOW_AUTOSIZE );
    setMouseCallback("Top View", mouseCrop, (void *)mouseData);
    
    if (mouseData->rec.size() == 1) {
        line(topImage, Point(mouseData->rec[0][0],mouseData->rec[0][1]), Point(mouseData->lastPoint.x, mouseData->rec[0][1]), Scalar(0,0,255));
        line(topImage, Point(mouseData->lastPoint.x, mouseData->lastPoint.y), Point(mouseData->lastPoint.x, mouseData->rec[0][1]), Scalar(0,0,255));
        line(topImage, Point(mouseData->lastPoint.x, mouseData->lastPoint.y), Point(mouseData->rec[0][0],mouseData->lastPoint.y), Scalar(0,0,255));
        line(topImage, Point(mouseData->rec[0][0],mouseData->rec[0][1]), Point(mouseData->rec[0][0],mouseData->lastPoint.y), Scalar(0,0,255));
    }
}

void TopView::setOrigin(Vec2f p){
    Vec3f P = convertToWorldCoord(Vec3f(0, 0, f));
    O = vecPlaneInter(P, convertToWorldCoord(Vec3f(p[0], p[1], f)));
}

void TopView::setScaleFactor(Vec2f a, Vec2f b, float dist){
    Vec3f P = convertToWorldCoord(Vec3f(0, 0, f));
    Vec3f A = vecPlaneInter(P, convertToWorldCoord(Vec3f(a[0], a[1], f)));
    Vec3f B = vecPlaneInter(P, convertToWorldCoord(Vec3f(b[0], b[1], f)));
    float d = pointDistance(A, B);
    
    sf = d/dist;
}

Vec2f TopView::toGroundPlaneCoord(Vec2f a){
    Vec3f P = convertToWorldCoord(Vec3f(0, 0, f));
    Vec3f A = vecPlaneInter(P, convertToWorldCoord(Vec3f(a[0], a[1], f)));
    
    A -= O;
    A /= sf;
    
    return Vec2f(A[0], A[1]);
}