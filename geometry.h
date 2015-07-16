//  Plane Projection
//  geometry.h
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifndef __UoB_Project__geometry__
#define __UoB_Project__geometry__

#include <stdio.h>

#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

void normalize_vec(Vec3f a);
Vec3f cross_product(Vec3f a, Vec3f b);
float dot_product(Vec3f a, Vec3f b);
float dot(Vec2f A, Vec2f B, Vec2f C);
float cross(Vec2f A, Vec2f B, Vec2f C);
double pointDistance(Vec2f A, Vec2f B);
double pointDistance(Vec3f A, Vec3f B);
double linePointDist(Vec2f A, Vec2f B, Vec2f C, bool isSegment);
float length(Vec3f A);
Vec3f vecPlaneInter(Vec3f p, Vec3f P);
void fitQuadRec(Point2f src[4], Point2f dst[4], Size size);
Vec2f meanSegmentIntersections(vector<Vec4f> segments);

#endif /* defined(__UoB_Project__geometry__) */