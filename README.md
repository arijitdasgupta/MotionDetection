Motion detection with OpenCV + Python
=====================================

I decided to make a general motion tracking system. I decided to make my own rudimentary motion tracking system to incorporate the camera rotation into the tracking scheme. There is an experimental branch which you can explore.

There is a another stepper motor based rotation surface built to track objects as they move, which is controlled by an ATmega8. The code is also included with source of the current branch.

The circuit to run the platform is fairly simple but I will add a schematic. I have plans to make the whole system better.

How to run
----------

To run the program you would need OpenCV 2.2 installed with Python 2.7. You also need a webcam (OpenCV compatible, most of them are). Simply run the .py file to run the program. In order to run the full system you would need to make the complete hardware. It's described in the C source file that is in the MotorControl folder.