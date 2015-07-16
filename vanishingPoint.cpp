//  Plane Projection
//  vanishingPoint.cpp
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "MSAC.h"

#include "TopView.h"
#include "geometry.h"
#include "vanishingPoint.h"

#include <iostream>

#define MAX_NUM_LINES	200

using namespace std;

void callbackFunc(int event, int x, int y, int flags, void* userdata){
    mouseDataVP *data = (mouseDataVP *) userdata;
    Mat temp = data->image.clone();
    
    if  (event == EVENT_LBUTTONDOWN ){
        if (data->clicked) {
            data->b = Point(x,y);
            
            if (!data->uDone)
                line(data->image, data->a, data->b, Scalar(0,0,255));
            else
                line(data->image, data->a, data->b, Scalar(0,255,0));
            
            data->clicked = false;
            imshow("Manual Calibration", data->image);
            
            if (!data->uDone)
                data->fumanual.push_back(Vec4i(data->a.x, data->a.y, data->b.x, data->b.y));
            else
                data->fvmanual.push_back(Vec4i(data->a.x, data->a.y, data->b.x, data->b.y));
        }
        else{
            data->a = Point(x,y);
            data->clicked = true;
        }
    }
    else if (event == EVENT_MOUSEMOVE ){
        if (data->clicked) {
            
            if (!data->uDone)
                line(temp, data->a, Point(x,y), Scalar(0,0,255));
            else
                line(temp, data->a, Point(x,y), Scalar(0,255,0));
            
            imshow("Manual Calibration", temp);
            temp = data->image.clone();
        }
        else{
            imshow("Manual Calibration", data->image);
        }
    }
    
    userdata = (void *) &data;
}

Vec4f manualCalibration(mouseDataVP *data){
    
    namedWindow("Manual Calibration");
    setMouseCallback("Manual Calibration", callbackFunc, (void *)data);
    imshow("Manual Calibration", data->image);
    
    cout << "Define at least two lines in the U direction and press SPACE when done." << endl;
    
    while (data->fumanual.size() < 2) {
        waitKey(0);
        
        if(data->fumanual.size() > 1){
            data->uDone = true;
            break;
        }
        else{
            cout << "ERROR: You must define at least " << 2 - data->fumanual.size() << " more line(s) in the U direction." << endl;
        }
    }
    
    cout << "Define at least two lines in the V direction and press SPACE when done." << endl;
    
    while (data->fvmanual.size() < 2) {
        waitKey(0);
        
        if(data->fvmanual.size() > 1){
            data->uDone = true;
            break;
        }
        
        else{
            cout << "ERROR: You must define at least " << 2 - data->fvmanual.size() << " more line(s) in the V direction." << endl;
        }
    }
    
    destroyWindow("Manual Calibration");
    
    Vec2f uvp = meanSegmentIntersections(data->fumanual);
    Vec2f vvp = meanSegmentIntersections(data->fvmanual);
    
    return Vec4f(uvp[0], uvp[1], vvp[0], vvp[1]);
}

/** This function contains the actions performed for each image*/
Vec4f automaticCalibration(MSAC &msac, int numVps, cv::Mat &imgGRAY, cv::Mat &outputImg, int houghThreshold)
{
    cv::Mat imgCanny;
    
    //equalizeHist(imgGRAY, imgGRAY);
    
    // Canny
    cv::Canny(imgGRAY, imgCanny, 200, 120, 3);
    
    // Hough
    vector<vector<cv::Point> > lineSegments;
    vector<cv::Point> aux;
    
    vector<Vec4i> lines;
    if(imgGRAY.cols*imgGRAY.rows < 400*400)
        houghThreshold = houghThreshold * (float)2/3;
    
    cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 80, 60);
    
    while(lines.size() > MAX_NUM_LINES)
    {
        lines.clear();
        houghThreshold += 10;
        cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10, 10);
    }
    for(size_t i=0; i<lines.size(); i++)
    {
        Point pt1, pt2;
        pt1.x = lines[i][0];
        pt1.y = lines[i][1];
        pt2.x = lines[i][2];
        pt2.y = lines[i][3];
        line(outputImg, pt1, pt2, CV_RGB(0,0,0), 2);
        /*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
         circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
         circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
         circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/
        
        // Store into vector of pairs of Points for msac
        aux.clear();
        aux.push_back(pt1);
        aux.push_back(pt2);
        lineSegments.push_back(aux);
    }
    
    // Multiple vanishing points
    
    std::vector<cv::Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
    std::vector<std::vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
    std::vector<int> numInliers;
    
    std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
    
    // Call msac function for multiple vanishing point estimation
    msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, numVps);
    for(int v=0; v<vps.size(); v++)
    {
        //printf("VP %d (%.3f, %.3f, %.3f)", v, vps[v].at<float>(0,0), vps[v].at<float>(1,0), vps[v].at<float>(2,0));
        fflush(stdout);
        double vpNorm = cv::norm(vps[v]);
        if(fabs(vpNorm - 1) < 0.001)
        {
            //printf("(INFINITE)");
            fflush(stdout);
        }
        //printf("\n");
    }
    
    // Draw line segments according to their cluster
    msac.drawCS(outputImg, lineSegmentsClusters, vps);
    
    if (vps.size() == 2)
        return Vec4f(vps[0].at<float>(0,0), vps[0].at<float>(1,0), vps[1].at<float>(0,0), vps[1].at<float>(1,0));
    else
        return Vec4f(-1,-1,-1,-1);
}

bool validVPS(Vec4f vps){
    return !(vps[0] == -1 && vps[1] == -1 && vps[2] == -1 && vps[3] == -1);
}