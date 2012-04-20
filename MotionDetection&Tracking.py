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
key_reset_upper = ord('R')
key_reset_lower = ord('r')

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
quality = 0.01 #cvGoodFeaturesTrack Quality factor
corner_count = 100 #Maximum corners to find with GoodFeaturesToTrack
min_distance = 7 #minimum distance between two corners
threshold_limit1_upper = 40 #Threshold for difference image calculation
threshold_limit1_lower = 20
fading_factor = 40 #Fading factor for sum image
threshold_limit2_lower = 100 #Threshold for sum image calculation
threshold_limit2_upper = 255
skip = 10 #Good Feature Skipper
param1 = 1 #Rotation parameter
detection_skip = 5 #Delay after a single movement
rotation_multiplier = 4 #Rotation per detection
filter_depth = 3 #Low pass moving average filter depth
cooloff_timer_limit = 10 #Motor cooloff timer limit

#Primary initialization
cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
cv.Copy(gray_image, prev_image)
corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track

#Initializing GoodFeatureToTrack execution skip counter
counter = 0

#misc initialization
flag = False #GoodFeatureToTrack execution flag
detection_skip_counter = 0
sum_of_avg = 0
cooloff_flag = False
cooloff_timer = 0
non_rotation_avg = 0

#Defining the rotation check function (for debugging purposes)
def rotation_check(x):
    if x == 'C':
        print "Did rotate"
    elif x == 'F':
        print "Didnt rotate"
        
def image_processor():
    cv.Copy(gray_image, register1_image)
    cv.Smooth(register1_image, register1_image, cv.CV_GAUSSIAN, 3, 3)
    cv.AbsDiff(register1_image, register2_image, accumulator)
    cv.InRangeS(accumulator, (threshold_limit1_lower), (threshold_limit1_upper), accumulator)
    cv.Dilate(accumulator, accumulator, None, 2)
    cv.Add(accumulator, sum_image, sum_image, accumulator)
    cv.SubS(sum_image, (fading_factor), sum_image)
    cv.InRangeS(sum_image, (threshold_limit2_lower), (threshold_limit2_upper), accumulator)
    cv.Copy(register1_image, register2_image)

def motion_detector():
    global corners, corner_count, quality, min_distance, flag, counter, avg
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

while True: #Main loop
    #Acquiring the image and grayscaling it
    img = cv.QueryFrame(capture)
    cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
    #Showing image
    if flag_true_image:
        cv.ShowImage(window1, accumulator)
    else:
        cv.ShowImage(window1, render_image)
    #Image processing
    image_processor()
    #Motion detection
    motion_detector()
    #detection_skip_counter decrement
    if detection_skip_counter > 0:
        detection_skip_counter = detection_skip_counter - 1
        non_rotation_avg = non_rotation_avg + avg
    elif detection_skip_counter == 0:
    #low pass moving average filtering for movement
        non_rotation_avg = non_rotation_avg/(detection_skip - 1)
    #printing movement and rotating the platform
    if avg > -non_rotation_avg and detection_skip_counter == 0:
        print "Movement right",
        if cooloff_flag:
            serial_port.write('I')
            cooloff_timer = 0
            x = serial_port.read()
            if x == 'i':
                serial_port.write('I')
                x = serial_port.read()
            cooloff_flag = False
        for i in range(rotation_multiplier):
            serial_port.write('R')
            x = serial_port.read()
        rotation_check(x)
        detection_skip_counter = detection_skip
        non_rotation_avg = 0
    elif avg < non_rotation_avg and detection_skip_counter == 0:
        print "Movement left",
        if cooloff_flag:
            serial_port.write('I')
            cooloff_timer = 0
            x = serial_port.read()
            if x == 'i':
                serial_port.write('I')
                x = serial_port.read()
            cooloff_flag = False
        for i in range(rotation_multiplier):
            serial_port.write('L')
            x = serial_port.read()
        rotation_check(x)
        detection_skip_counter = detection_skip
        non_rotation_avg = 0
    elif avg < param1 and avg > -param1:
        if cooloff_timer < cooloff_timer_limit:
            cooloff_timer = cooloff_timer + 1
        elif not cooloff_flag:
            serial_port.write('I')
            x = serial_port.read()
            print 'Motor cooling off'
            cooloff_flag = True
    #Runtime keystroke controls with flags
    key = cv.WaitKey(1)
    if(key == key_quit_lower or key == key_quit_upper):
        cv.DestroyWindow(window1)
        serial_port.write('I');
        sleep(0.1)
        x = serial_port.read()
        if x == 'I':
            serial_port.write('I')
            x = serial_port.read()
            if x == 'i':
                print "System set to stand-by mode and motor cooloff"
        serial_port.close()
        exit()
    elif(key == key_true_image_upper or key == key_true_image_lower):
        flag_true_image = not flag_true_image
    elif(key == key_print_upper or key == key_print_lower):
        print len(new_corners), len(corners), len(status), len(track_error)
    elif(key == key_reset_upper or key == key_reset_lower):
        print 'Resetting camera angle'
        x = 'A'
        serial_port.write('I')
        if serial_port.read() == 'i':
            serial_port.write('I')
            serial_port.read()
        while x != 'F':
            serial_port.write('R')
            x = serial_port.read()
        for i in range(100):
            serial_port.write('L')
            x = serial_port.read()
        sleep(0.2)