//  Plane Projection
//  geometry.cpp
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#include "geometry.h"

#include <iostream>

void normalize_vec(Vec3f a){
    float l = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    a[0] /= l;
    a[1] /= l;
    a[2] /= l;
}

Vec3f cross_product(Vec3f a, Vec3f b){
    Vec3f result;
    result[0] = a[1] * b[2] - b[1] * a[2];
    result[1] = a[2] * b[0] - b[2] * a[0];
    result[2] = a[0] * b[1] - b[0] * a[1];
    return result;
}

float dot_product(Vec3f a, Vec3f b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

//Compute the dot product AB ⋅ BC
float dot(Vec2f A, Vec2f B, Vec2f C){
    Vec2f AB(B[0] - A[0], B[1] - A[1]);
    Vec2f BC(C[0] - B[0], C[1] - B[1]);
    float dot = AB[0] * BC[0] + AB[1] * BC[1];
    return dot;
}
//Compute the cross product AB x AC
float cross(Vec2f A, Vec2f B, Vec2f C){
    Vec2f AB(B[0] - A[0], B[1] - A[1]);
    Vec2f AC(C[0] - A[0], C[1] - A[1]);
    float cross = AB[0] * AC[1] - AB[1] * AC[0];
    return cross;
}
//Compute the distance from A to B
double pointDistance(Point2f A, Point2f B){
    Vec2f d = A - B;
    return length(d);
}

double pointDistance(Point3f A, Point3f B){
    Vec3f d = A - B;
    return length(d);
}

//Compute the distance from AB to C
//if isSegment is true, AB is a segment, not a line.
double linePointDist(Point2f A, Point2f B, Point2f C, bool isSegment){
    Vec2f a(A.x, A.y);
    Vec2f b(B.x, B.y);
    Vec2f c(C.x, C.y);
    
    double dist = cross(a, b, c) / pointDistance(a, b);
    if(isSegment){
        float dot1 = dot(a, b, c);
        if(dot1 > 0)return pointDistance(b, c);
        float dot2 = dot(b, a, c);
        if(dot2 > 0)return pointDistance(a, c);
    }
    return abs(dist);
}

float length(Vec3f A){
    return sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
}

float length(Vec2f A){
    return sqrt(A[0]*A[0] + A[1]*A[1]);
}

//intersection between vector and ground plane
Point3f vecPlaneInter(Vec3f p, Point3f P){
    Vec3f normal(0,0,1);
    float D = - normal[0] * p[0] - normal[1] * p[1] - normal[2] * p[2];
    
    Vec3f dir(P.x, P.y, P.z);
    normalize_vec(dir);
    
    double t = -(D)/(normal[0] * dir[0] + normal[1] * dir[1] + normal[2] * dir[2]);
    
    return t * dir;
}

//fits a poligon into a quadrilateral
void fitQuadRec(Point2f src[4], Point2f dst[4], Size size){
    Point2f min( 9999999,  9999999);
    Point2f max(-9999999, -9999999);
    
    for (int i = 0; i < 4; i++){
        if(src[i].x < min.x)
            min.x = src[i].x;
        if(src[i].y < min.y)
            min.y = src[i].y;
        if(src[i].x > max.x)
            max.x = src[i].x;
        if(src[i].y > max.y)
            max.y = src[i].y;
    }
    
    float ratio;
    
    if ((max.y - min.y) / ((max.x - min.x)/size.width) <= size.height)
        ratio = (max.x - min.x)/size.width;
    else
        ratio = (max.y - min.y)/size.height;
    
    dst[0] = Point2f((src[0].x - min.x)/ratio, (src[0].y - min.y)/ratio);
    dst[1] = Point2f((src[1].x - min.x)/ratio, (src[1].y - min.y)/ratio);
    dst[2] = Point2f((src[2].x - min.x)/ratio, (src[2].y - min.y)/ratio);
    dst[3] = Point2f((src[3].x - min.x)/ratio, (src[3].y - min.y)/ratio);
}

Vec2f meanSegmentIntersections(vector<Vec4f> segments){
    float a1, a2, b1, b2, x, y;
    vector<Vec2f> intersections;
    
    //calculate intersections
    for( int i = 0; i < segments.size(); i++){
        a1 = (segments[i][0] - segments[i][2])/(segments[i][1] - segments[i][3]);
        b1 = segments[i][0] - a1 * segments[i][1];
        
        for( size_t j = 0; j < segments.size(); j++){
            if(i != j){
                
                a2 = (segments[j][0] - segments[j][2])/(segments[j][1] - segments[j][3]);
                b2 = segments[j][0] - a2 * segments[j][1];
                //parallel
                if (a1 - a2 != 0) {
                    y = (b2 - b1)/(a1 - a2);
                    x = a1 * y + b1;
                    intersections.push_back(Vec2f(x, y));
                }
            }
        }
    }
    
    Vec2f mean(0,0);
    for (int i = 0; i < intersections.size(); i++){
        mean += intersections[i];
    }
    
    mean /= (int)intersections.size();
    
    return mean;
}