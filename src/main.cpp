//  Plane Projection
//  main.cpp
//
//  University of Bristol
//
//  Created by Henrique Grandinetti on 13/07/15.
//  henriquegrandinetti@gmail.com

#ifdef WIN32
#include <windows.h>
#endif
#include <iostream>
#ifdef linux
#include <stdio.h>
#endif

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "MSAC.h"

#include "TopView.h"
#include "geometry.h"
#include "vanishingPoint.h"

using namespace std;
using namespace cv;

void help(){
    cout
    << " ------------------------------------------------------------------------\n"
    << " | Usage: \n"
    << " |		-video		: Video file as input (Default: camera) \n"
    << " |		-image		: Image file as input (Default: camera) \n"
    << " |		-still		: Camera doesn't change position, for a more stable projection \n"
    << " |		-manual		: Manual calibration of vanishing points \n"
    << " |		-play		: ON: the video runs until the end; OFF: frame by frame (key press event)\n"
    << " |		-resizedWidth	: Width size (Height calculated based on aspect ratio)\n"
    << " |		-houghThreshold	: Threshold for finding lines. Bigger less lines, smaller more lines. (Default: 120)\n"
    << " | Keys:\n"
    << " |		Esc: Quit\n"
    << " -------------------------------------------------------------------------\n"
    << endl;
}

/* ----------------------------------------
reads points from file.
The points should be in the following format:
[x1,y1], [x2, y2], ... 
-------------------------------------------*/
void readPointsFile(string fileName, vector<Point2f> *points){
    ifstream file(fileName.c_str());
    string line;
    
    while(getline(file, line)) {
        float x, y;
        char open_br, close_br, comma;
        stringstream line_stream(line);
        while(line_stream >> open_br >> x >> comma >> y >> close_br >> comma){
            points->push_back(Point2f(x, y));
        }
        line_stream >> open_br >> x >> comma >> y >> close_br;
        points->push_back(Point2f(x, y));
    }
}

/** Main function*/
int main(int argc, char** argv)
{
    Vec2f Fu, Fv;
    
    cv::Mat inputImg, imgGRAY;
    cv::Mat outputImg;
    
    cv::VideoCapture video;
    cv::Size procSize;
    
    char *videoFileName = 0;
    char *imageFileName = 0;
    
    int procWidth = -1;
    int procHeight = -1;
    int numVps = 2;
    int numFramesCalib = 40;
    int numFramesSmooth = 30;
    int houghThreshold = 120;
    
    bool useCamera = true;
    bool playMode = true;
    bool stillImage = false;
    bool stillVideo = false;
    bool manual = false;
    
    //variable to print a trajectory
    //vector<Point2f> trajectories;
    //readPointsFile("trajectories.txt", &trajectories);
    
    // Parse arguments
    for(int i=1; i<argc; i++){
        
        const char* s = argv[i];
        
        if(strcmp(s, "-video" ) == 0){
            // Input video is a video file
            videoFileName = argv[++i];
            useCamera = false;
        }
        else if(strcmp(s,"-image") == 0){
            // Input is a image file
            imageFileName = argv[++i];
            stillImage = true;
            useCamera = false;
        }
        else if(strcmp(s, "-resizedWidth") == 0){
            procWidth = atoi(argv[++i]);
        }
        else if(strcmp(s, "-still" ) == 0){
            const char* ss = argv[++i];
            if(strcmp(ss, "ON") == 0 || strcmp(ss, "on") == 0
               || strcmp(ss, "TRUE") == 0 || strcmp(ss, "true") == 0
               || strcmp(ss, "YES") == 0 || strcmp(ss, "yes") == 0 )
                stillVideo = true;
        }
        else if(strcmp(s, "-manual" ) == 0){
            const char* ss = argv[++i];
            if(strcmp(ss, "ON") == 0 || strcmp(ss, "on") == 0
               || strcmp(ss, "TRUE") == 0 || strcmp(ss, "true") == 0
               || strcmp(ss, "YES") == 0 || strcmp(ss, "yes") == 0 )
                manual = true;
        }
        else if(strcmp(s, "-play" ) == 0){
            const char* ss = argv[++i];
            if(strcmp(ss, "OFF") == 0 || strcmp(ss, "off") == 0
               || strcmp(ss, "FALSE") == 0 || strcmp(ss, "false") == 0
               || strcmp(ss, "NO") == 0 || strcmp(ss, "no") == 0
               || strcmp(ss, "STEP") == 0 || strcmp(ss, "step") == 0)
                playMode = false;
        }
        else if(strcmp(s, "-houghThreshold") == 0){
            houghThreshold = atoi(argv[++i]);
        }
        else if(strcmp(s, "-help" ) == 0){
            help();
        }
    }
    
    // Open video input
    if(useCamera)
        video.open(0);
    else{
        if(!stillImage)
            video.open(videoFileName);
    }
    
    // Check video input
    int width = 0, height = 0, fps = 0, fourcc = 0;
    if(!stillImage){
        if( !video.isOpened() ){
            printf("ERROR: can not open camera or video file\n");
            return -1;
        }
        else{
            // Show video information
            width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH);
            height = (int) video.get(CV_CAP_PROP_FRAME_HEIGHT);
            fps = (int) video.get(CV_CAP_PROP_FPS);
            fourcc = (int) video.get(CV_CAP_PROP_FOURCC);
            
            if(!useCamera)
                printf("Input video: (%d x %d) at %d fps, fourcc = %d\n", width, height, fps, fourcc);
            else
                printf("Input camera: (%d x %d) at %d fps\n", width, height, fps);
        }
    }
    else{
        inputImg = cv::imread(imageFileName);
        if(inputImg.empty())
            return -1;
        
        width = inputImg.cols;
        height = inputImg.rows;
        
        printf("Input image: (%d x %d)\n", width, height);
        
        playMode = false;
    }
    
    // Resize
    if(procWidth != -1){
        
        procHeight = height*((double)procWidth/width);
        procSize = cv::Size(procWidth, procHeight);
        
        printf("Resize to: (%d x %d)\n", procWidth, procHeight);
    }
    else
        procSize = cv::Size(width, height);
    
    // Create and init MSAC
    MSAC msac;
    msac.init(procSize);
    
    //create mouse structs
    mouseDataCrop mdCrop;
    mdCrop.windowName = "Top View"; //topview window name
    mouseDataVP mdVP;
    mdVP.uDone = false;
    mdVP.clicked = false;
    
    Vec4f previousVP;
    Vec4f averageVP;
    Vec4f vp;
    vector<Vec4f> vpVector;
    vector<Vec4f> stillVPS;
    
    bool averageCompleted = false;
    
    int frameNum=0;
    for(;;){
        
        if(!stillImage)
        {
            frameNum++;
            
            //Get current image
            video >> inputImg;
        }
        
        if(inputImg.empty())
            break;
        
        //Resize to processing size
        cv::resize(inputImg, inputImg, procSize);
        
        //Color Conversion
        if(inputImg.channels() == 3){
            cv::cvtColor(inputImg, imgGRAY, CV_BGR2GRAY);
            inputImg.copyTo(outputImg);
        }
        else{
            inputImg.copyTo(imgGRAY);
            cv::cvtColor(inputImg, outputImg, CV_GRAY2BGR);
        }
        
        ////////////////////////////
        // Calculate VPs
        ////////////////////////////
        
        
        //manual calibration
        if(manual && frameNum == 3){
            mdVP.image = inputImg.clone();
            vp = manualCalibration(&mdVP);
        }
        
        //still video
        else if(!manual && stillVideo){
            //add vp to vector
            if (frameNum < numFramesCalib && !averageCompleted) {
                vp = automaticCalibration(msac, numVps, imgGRAY, outputImg, houghThreshold);
                if (validVPS(vp))
                    stillVPS.push_back(vp);
            }
            
            //average vp, re-start video and zero frame num
            else if(frameNum == numFramesCalib && !averageCompleted){
                for (int i = 0; i < stillVPS.size(); i++) {
                    vp += stillVPS[i];
                }
                vp /= (int)stillVPS.size();
                
                if (!useCamera) {
                    video.open(videoFileName);
                    frameNum = 0;
                    averageCompleted = true;
                    continue;
                }
            }
        }
        
        //automatic calibration
        if (!manual && !stillVideo){
            vp = automaticCalibration(msac, numVps, imgGRAY, outputImg, houghThreshold);
            
            //smooth vp position
            if (vpVector.size() < numFramesSmooth)
                vpVector.push_back(vp);
            
            else if(!stillVideo){
                vpVector.erase(vpVector.begin());
                vpVector.push_back(vp);
                averageVP = Vec4f(0,0,0,0);
                for (int i = 0; i < numFramesSmooth; i++) {
                    averageVP += vpVector[i];
                }
                averageVP /= numFramesSmooth;
                vp = averageVP;
            }
        }
        
        //avoid vps to swap position
        if (frameNum != 0 &&
            pointDistance(Point2f(previousVP[0], previousVP[1]), Point2f(vp[0],vp[1])) > pointDistance(Point2f(previousVP[0], previousVP[1]), Point2f(vp[2],vp[3])) &&
            pointDistance(Point2f(previousVP[2], previousVP[3]), Point2f(vp[2],vp[3])) > pointDistance(Point2f(previousVP[2], previousVP[3]), Point2f(vp[0],vp[1]))){
            
            Vec4f temp(vp);
            vp[0] = vp[2];
            vp[1] = vp[3];
            vp[2] = temp[0];
            vp[3] = temp[1];
            
        }
        
        previousVP = Vec4f(vp);
        
        ////////////////////////////
        // Calculate Top-View
        ////////////////////////////
        
        if (validVPS(vp)){
            
            Fu = Point2f(vp[0], vp[1]);
            Fv = Point2f(vp[2], vp[3]);
                        
            TopView tv(inputImg, Fu, Fv, &mdCrop);
            tv.drawAxis(outputImg, Point(0,0));
            
            tv.generateTopImage();
            
            //allows to crop top view
            tv.cropTopView();
            
            tv.setOrigin(Point(444,325));
            tv.setScaleFactor(Point(444,325), Point(505, 149), 5.0);
            //Point P = tv.toGroundPlaneCoord(Point(464, 268));
            
            /*vector<Point2f> b;
            b = tv.toTopViewCoordinates(trajectories);
            
            for (int k = 0; k < b.size(); k++){
                circle(tv.topImage, b[k], 2, Scalar(255,0,0));
                circle(outputImg, trajectories[k], 2, Scalar(255,0,0));
            }*/
            
            imshow(mdCrop.windowName, tv.topImage);
        }
        
        imshow("Original", outputImg);
        
        if(playMode)
            cv::waitKey(1);
        else
            cv::waitKey(0);
        
        char q = (char)waitKey(1);
        
        if( q == 27 ){
            printf("\nStopped by user request\n");
            break;
        }
        
        if(stillImage)
            break;
    }
    
    if(!stillImage)
        video.release();
    
    return 0;
}