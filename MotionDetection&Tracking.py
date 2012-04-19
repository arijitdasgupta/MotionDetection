#Universal motion detection algorithm implementation
#General motion detection using GoodFeaturesToTrack and Pyramidal Lucas Kanade
#Author: Arijit Dasgupta

import cv
import serial
from time import sleep

#Key codes for initialization faster processing
key_quit_lower = ord('q')
key_quit_upper = ord('Q')
key_true_image_lower = ord('b')
key_true_image_upper = ord('B')
key_print_upper = ord('P')
key_print_lower = ord('p')

#Viewing flags
flag_true_image = False

#Initialzing window for rendering
window1 = 'main_window'
cv.NamedWindow(window1,1)
capture = cv.CaptureFromCAM(0)

#Initializing camera and stuff
try:
    img = cv.QueryFrame(capture)
    if img == None:
        print "Couldn't get the camera, exiting program"
        exit()
except:
    print "Error getting image camera, exiting program"
    exit()

print "Initialized camera..."
    
#Initialzing serial port
try:
    serial_port = serial.Serial(14) #Opening COM15 as BlueLink, you should change as per your serial port
    serial_port.write('I') #Initializing the system
    sleep(0.1)
    x = serial_port.read()
    if x == 'I':
        print "System initialized... commencing tracking program"
    else:
        print "Failed to initialize motor system, trying again"
        serial_port.write('I') #Initializing the system again
        sleep(0.1)
        x = serial_port.read()
        if x == 'I':
            print "System initialized... commencing tracking program"
        else:
            print "Failed to initialize motor system"
            exit()
except:
    print "Error opening serial port"
    exit()

#Initializing images
eigen_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_32F, 1) #Eigen image for Good Features to track
temp_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_32F, 1) #For good feature to track
gray_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #grayscale image
prev_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #previous image store
render_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 3) #image to render in the end
accumulator = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #accumulator
register1_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #Image processing register 1
register2_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #Image processing register 2
sum_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #Sum image register

pyramid1 = cv.CreateImage([img.width + 8, img.height/3], cv.IPL_DEPTH_32F, 1) #Pyramid buffers of Pyramidal Lucas Kanade Method
pyramid2 = cv.CreateImage([img.width + 8, img.height/3], cv.IPL_DEPTH_32F, 1)

#hardcoded optimizable params, might be implemented with a slider afterwards
quality = 0.1 #cvGoodFeaturesTrack Quality factor
corner_count = 50 #Maximum corners to find with GoodFeaturesToTrack
min_distance = 10 #minimum distance between two corners
threshold_limit1_upper = 40 #Threshold for difference image calculation
threshold_limit1_lower = 20
fading_factor = 40 #Fading factor for sum image
threshold_limit2_lower = 100 #Threshold for sum image calculation
threshold_limit2_upper = 255
skip = 10 #Good Feature Skipper
param1 = 2 #Rotation parameter

#Primary initialization
cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
cv.Copy(gray_image, prev_image)
corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track

#Initializing GoodFeatureToTrack execution skip counter
counter = 0

#misc initialization
flag = False #GoodFeatureToTrack execution flag

while True: #Main loop
    #Acquiring the image
    img = cv.QueryFrame(capture)
    #Showing image
    if flag_true_image:
        cv.ShowImage(window1, gray_image)
    else:
        cv.ShowImage(window1, render_image)
    #Image processing
    cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
    cv.Copy(gray_image, register1_image)
    cv.Smooth(register1_image, register1_image, cv.CV_GAUSSIAN, 3, 3)
    cv.AbsDiff(register1_image, register2_image, accumulator)
    cv.InRangeS(accumulator, (threshold_limit1_lower), (threshold_limit1_upper), accumulator)
    cv.Dilate(accumulator, accumulator, None, 2)
    cv.Add(accumulator, sum_image, sum_image, accumulator)
    cv.SubS(sum_image, (fading_factor), sum_image)
    cv.InRangeS(sum_image, (threshold_limit2_lower), (threshold_limit2_upper), accumulator)
    cv.Copy(register1_image, register2_image)
    #Motion detection
    new_corners, status, track_error = cv.CalcOpticalFlowPyrLK(prev_image, gray_image, pyramid1, pyramid2, corners, (10,10), 2, (cv.CV_TERMCRIT_ITER, 10, 0), 0)
    counter = (counter + 1) % skip
    if(counter == 0):
        corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track
        flag = True
    cv.Copy(img, render_image)
    cv.Copy(img, render_image)
    cv.Copy(gray_image, prev_image)
    #Drawing vectors and averaging the rotation...
    sum = 0
    summing_counter = 0
    if flag:
        flag = not flag
    else:
        for i in range(len(new_corners)):
            try: #Have to fix the exception, for now, it's running by ignoring the index error...
                x1, y1 = corners[i]
                x1, y1 = int(x1), int(y1)
                x2, y2 = new_corners[i]
                x2, y2 = int(x2), int(y2)
                avg = 0
                if (cv.Get2D(accumulator, y1, x1)[0] >= 240):
                    cv.Line(render_image, (x1, y1), (x2, y2), (0,255,255))
                    cv.Circle(render_image, (x2, y2), 1, (0,255,0))
                    sum = sum + (x2 - x1)
                    summing_counter = summing_counter + 1
                    avg = sum/summing_counter
            except IndexError:
                pass
    #printing movement
    if avg > param1:
        print "Movement right"
    elif avg < -param1:
        print "Movement left"
    if avg > param1:
        serial_port.write('R')
        x = serial_port.read()
        if x == 'C':
            print 'Rotated right'
            corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track
        elif x == 'F':
            print 'Didnt rotate right'
    if avg > -param1:
        serial_port.write('L')
        x = serial_port.read()
        if x == 'C':
            print 'Rotated left'
            corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track
        elif x == 'F':
            print 'Didnt rotate left'
    #Runtime keystroke controls with flags
    key = cv.WaitKey(1)
    if(key == key_quit_lower or key == key_quit_upper):
        cv.DestroyWindow(window1)
        serial_port.write('I');
        sleep(0.1)
        x = serial_port.read()
        if x == 'i':
            print "System set to stand-by mode cooloff"
        serial_port.close()
        exit()
    elif(key == key_true_image_upper or key == key_true_image_lower):
        flag_true_image = not flag_true_image
    elif(key == key_print_upper or key == key_print_lower):
        print len(new_corners), len(corners), len(status), len(track_error)