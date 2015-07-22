Automatic Camera Calibration for Top-View Projection

version 1.0 - July 2015

======================================================

Introduction:
-------------

This software automatically calibrates a camera based on two vanishing points from the image and generates a top-view projection of the plane defined by these same vanishing points. No camera parameter is needed.

The code has also some built-in functions that allows to make measurements in the detected plane.

Requirements:
-------------

This software requires the OpenCV library.

It was successfully tested with Opencv 2.4.

Installation:
-------------

* The first thing is to make sure you have CMake installed in your computer, if not you can download it [here](http://www.cmake.org/download/). In linux "$ apt-get install cmake" will also work.

* After installed cmake run the following commands:

  $ cmake .

  $ make

* Now, to run the program simply type "$ ./ACCTVP [options]"

Executable Options:
-------------------

-video  <path>
Uses a video as input. (Default: camera)

-image  <path>
Uses a image as input. (Default: camera)

-still  <bool>
To be used when camera doesn't change position during recording, the software will provide a more stable projection. The vanishing points will be calculated in 40 frames and averaged, then the average vanishing points will be used for the whole video. (Default: false)

-manual <bool>
The vanishing point estimation will be done manually by the user, that has to determine at least two parallel lines for each of the two axis that defines the plane of interest. It is important to notice that the vanishing points will be kept the same during the whole video, therefore the camera must not be changing position. (Default: false)

-play	<ON/OFF>
ON: The video runs until the end; OFF: frame by frame (key press event). (Default: OFF)

-resizedWidth   <integer>
Resizes the image width, height is calculated based on aspect ratio.

-houghThreshold	<integer>
Threshold for finding lines that will determine the vanishing points. Less lines are found as the threshold increases and more lines as it decreases. (Default: 120)

Usage Examples:
---------------

$ ./ACCTVP -resizedWidth 600 -video footage1.mov
$ ./ACCTVP -image photo1.jpeg
$ ./ACCTVP -video footage1.mov -manual true -play ON
$ ./ACCTVP -resizedWidth 600 -video footage1.mov -houghThreshold 150

Plane Measurements with TopView Class:
--------------------------------------

* The TopView class is responsible not only to generate the top-view image but also to allow the plane measuraments that I have mentioned in the introduction. Here I will explain what each function does and how to use it.

-- void drawAxis(Mat output, Point p);
Draws the world axis on the image "output", centered at the point "p".

-- void generateTopImage();
Generates the top-image and stores it in the property "topImage".

-- vector<Vec2f> toTopViewCoordinates(vector<Vec2f> a);
Transforms points (Vec2f) from the original image coordinates to the top-image coordinates.

-- void cropTopView();
Allows the top-view image to be croped to a smaller region of interest.

* Before mentioning the next functions, it is important to say that without any information from the real world all the measurements are provided under a scale factor, and the world space origin will be in the center of the camera sensor. It is, however, possible to change this with the following functions.

-- void setOrigin(Vec2f p);
Defines a new origin to the world space. The point p, in image coordinates will be this new point.

-- void setScaleFactor(Vec2f a, Vec2f b, float dist);
To have the measurements in a known scale it is needed to set the scale factor. This can be done with this function providing two points "a" and "b" in image coordinates, and the real distance "dist" between them.

-- Vec2f toGroundPlaneCoord(Vec2f a);
Gives the ground plane coordinate of a point "a" in image coordinates.

License:
--------

This is a free to use for non-commercial purposes software.

The automatic vanishing point estimation is based on the software created by Marcos Nieto and can be found in the original version [here](www.marcosnieto.net/vanishingPoint).

The Least Squares Fitting method used by Nieto was created by Joachim Wuttke and can be found in its original version [here](www.messen-und-deuten.de/lmfit).

Contact:
--------

Henrique Grandinetti - henriquegrandinetti@gmail.com