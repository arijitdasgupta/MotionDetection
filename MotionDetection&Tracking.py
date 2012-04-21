#Universal motion detection algorithm implementation
#Largest contour detection analysis
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
gray_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #grayscale image
prev_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #previous image store
temp_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
render_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 3) #image to render in the end
accumulator = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #accumulator
sum_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1) #Sum image register

#hardcoded optimizable params, might be implemented with a slider afterwards
threshold_limit1_upper = 40 #Threshold for difference image calculation
threshold_limit1_lower = 20
fading_factor = 150 #Fading factor for sum image
threshold_limit2_lower = 100 #Threshold for sum image calculation
threshold_limit2_upper = 255
detection_skip = 5 #Delay after a single movement
rotation_multiplier = 4 #Rotation per detection
filter_depth = 10 #Low pass moving average filter depth
cooloff_timer_limit = 10 #Motor cooloff timer limit
max_area = 1000 #min area for a difference image contour

#Primary initialization
cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
cv.Copy(gray_image, prev_image)

#Initializing store for contour detection
store = cv.CreateMemStorage()

#misc initialization
flag = False #GoodFeatureToTrack execution flag
detection_skip_counter = 0
cooloff_flag = False
cooloff_timer = 0
avg = 0
prev_pos = 0

#Defining the rotation check function (for debugging purposes)
def rotation_check(x):
    if x == 'C':
        print "Did rotate"
    elif x == 'F':
        print "Didnt rotate"
        
def image_processor():
    cv.Smooth(gray_image, gray_image, cv.CV_GAUSSIAN, 3, 3)
    cv.AbsDiff(prev_image, gray_image, accumulator)
    cv.InRangeS(accumulator, threshold_limit1_lower, threshold_limit1_upper, accumulator)
    cv.Dilate(accumulator, accumulator, None, 2)
    cv.Add(accumulator, sum_image, sum_image, accumulator)
    cv.SubS(sum_image, fading_factor, sum_image)
    cv.InRangeS(sum_image, threshold_limit2_lower, threshold_limit2_upper, accumulator)
    cv.Copy(gray_image, prev_image)
    cv.Copy(accumulator, temp_image)

def motion_detector():
    global max_area, avg, prev_pos, largest_contour
    contour = cv.FindContours(temp_image, store, mode = cv.CV_RETR_EXTERNAL, method = cv.CV_CHAIN_APPROX_NONE)
    if len(contour) != 0:
        temp_contour = contour
        area = 0
        max_area_test = max_area
        while temp_contour != None:
            area = cv.ContourArea(temp_contour)
            if area > max_area_test:
                largest_contour = temp_contour
                max_area_test = area
            temp_contour = temp_contour.h_next()
        rect = cv.BoundingRect(largest_contour)
        cv.DrawContours(img, largest_contour, (0,255,0), (0,0,255), 1, 3)
        cv.Rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (255,0,0))
        avg = rect[0] + rect[2]/2
    else:
        avg = img.width/2
    cv.Copy(img, render_image)

while True: #Main loop
    #Acquiring the image and grayscaling it
    img = cv.QueryFrame(capture)
    cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
    #Showing image
    if flag_true_image:
        cv.ShowImage(window1, render_image)
    else:
        cv.ShowImage(window1, accumulator)
    #Image processing
    image_processor()
    #Motion detection
    motion_detector()
    #detection_skip_counter decrement
    if detection_skip_counter > 0:
        detection_skip_counter = detection_skip_counter - 1
    #printing movement and rotating the platform
    if avg > 2*(img.width/3) and detection_skip_counter == 0:
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
    elif avg < img.width/3 and detection_skip_counter == 0:
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
    else:
        if cooloff_timer < cooloff_timer_limit:
            cooloff_timer = cooloff_timer + 1
        elif not cooloff_flag:
            serial_port.write('I')
            x = serial_port.read()
            print 'Motor cooling on'
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
                print "System set to stand-by mode and motor cooling on"
        serial_port.close()
        exit()
    elif(key == key_true_image_upper or key == key_true_image_lower):
        flag_true_image = not flag_true_image
    elif(key == key_print_upper or key == key_print_lower):
        print largest_contour[:]
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