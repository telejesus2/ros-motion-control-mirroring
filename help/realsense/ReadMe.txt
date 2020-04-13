The C++ program record.cpp records synchronized depth and RGB images with the intel realsense camera D435.

Dependencies
============
Opencv and librealsense2

How to use it under Linux
=========================
1. Create a folder named "data" in the same directory of record.cpp and stb_image_write.h.
The frames will be stored in the folder "data".
If this folder doesn't existed, frame storage will fail.

2. Compile
g++ -std=c++11 record.cpp -o record `pkg-config --cflags --libs opencv` -lrealsense2

3. Run
./record

What happens when you run the binary
====================================
in the folder named data, for each time step (depending on the chosen frame rate), the following data will be stored:
- an RGB image
- a deph image
- a .csv file containing metadata about the RGB frame
- a .csv file containing metadata about the depth frame
In the console the number of frames are displayed.

How to stop the recording
=========================
press Ctrl+C
