#Automatic Camera Calibration for Top-View Projection

This software allows measurements and generates the top view projection of a plane in a single view image. The plane is defined based on two vanishing points in the image, that can be both defined manually or automatically.

##Installation

* The first thing is to make sure you have CMake installed in your computer, if not you can download it [here](http://www.cmake.org/download/). In linux `apt-get install cmake` will also work.

* After installed cmake run the following commands:

  `cmake .`

  `make`

* Now, to run the program simply type `./planeProjection`

##License

This is a free to use for non-commercial purposes software.

The automatic vanishing point estimation is based on the software created by Marcos Nieto and can be found in the original version [here](www.marcosnieto.net/vanishingPoint).

The Least Squares Fitting method used by Nieto was created by Joachim Wuttke and can be found in its original version [here](www.messen-und-deuten.de/lmfit).