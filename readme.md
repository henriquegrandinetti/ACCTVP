#Automatic Camera Calibration for Top-View Projection

v1.0

This software automatically calibrates a camera based on two vanishing points from the image and generates a top-view projection of the plane defined by these same vanishing points. No camera parameter is needed.

The code has also some built-in functions that allows to make measurements in the detected plane.

##Installation

* The first thing is to make sure you have CMake installed in your computer, if not you can download it [here](http://www.cmake.org/download/). In linux `apt-get install cmake` will also work.

* After installed cmake run the following commands:

  `cmake .`

  `make`

* Now, to run the program simply type `./ACCTVP`

##Requirements

This software requires the OpenCV library.

It was successfully tested with OpenCV 2.4.

##Demo

A demo can be seen at https://www.youtube.com/watch?v=wzTJg9jUTHg.

##License

This is a free to use for non-commercial purposes software.

The automatic vanishing point estimation is based on the software created by Marcos Nieto and can be found in the original version [here](www.marcosnieto.net/vanishingPoint).

The Least Squares Fitting method used by Nieto was created by Joachim Wuttke and can be found in its original version [here](www.messen-und-deuten.de/lmfit).

##More Info

More information available on readme.txt

##Contact

Henrique Grandinetti - henriquegrandinetti@gmail.com