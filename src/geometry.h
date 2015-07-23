//  Plane Projection
//  geometry.h
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifndef __ACCTVP__geometry__
#define __ACCTVP__geometry__

#include <stdio.h>

#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

void normalize_vec(Vec3f a);
Vec3f cross_product(Vec3f a, Vec3f b);
float dot_product(Vec3f a, Vec3f b);
float dot(Vec2f A, Vec2f B, Vec2f C);
float cross(Vec2f A, Vec2f B, Vec2f C);
double pointDistance(Point2f A, Point2f B);
double pointDistance(Point3f A, Point3f B);
double linePointDist(Point2f A, Point2f B, Point2f C, bool isSegment);
float length(Vec3f A);
float length(Vec2f A);
Point3f vecPlaneInter(Vec3f p, Point3f P);
void fitQuadRec(Point2f src[4], Point2f dst[4], Size size);
Vec2f meanSegmentIntersections(vector<Vec4f> segments);

#endif