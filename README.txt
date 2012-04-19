Motion detection with OpenCV + Python
-------------------------------------

This starts as a B. Tech final year project in object tracking. Most internet references I got my hands on was based on color based object tracking. I decided to make a general motion tracking system.

I tried using Optical Flow calculation methods that comes with OpenCV, but I found them not easy to use, so I decided to make my own rudimentary motion tracking system.
I plan to extend the whole program, for now it can detect stuff moving left or right with a webcam and prints the direction out.

There is a another rotation surface built to track objects as they move, which is controlled by an AVR ATmega8. The code is also included with source of the current branch.

To run the program you would need OpenCV 2.2 installed with Python 2.7

There is also a dead branch named difference_image_averaging, which incorporated my own algorithm for motion detection of the largest moving object on screen. The branch is dead. Current version is using Lucas Kanade Optical Flow Detection Method. You are also encourage to check out the dead branch.