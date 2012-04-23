Motion detection with OpenCV + Python
-------------------------------------

This starts as a B. Tech final year project in object tracking. Most internet references I got my hands on was based on color based object tracking. I decided to make a general motion tracking system.

I tried using Optical Flow calculation methods that comes with OpenCV, but I found them not easy to use, so I decided to make my own rudimentary motion tracking system. There is an experimental branch which you can explore.
I do plan to extend the whole program, for now it can detect stuff moving left or right with a webcam and prints the direction out.

There is a another stepper motor based rotation surface built to track objects as they move, which is controlled by an ATmega8. The code is also included with source of the current branch.

The circuit to run the platform is fairly simple but I will add a schematic soon.

To run the program you would need OpenCV 2.2 installed with Python 2.7.
