#Universal motion detection algorithm implementation
#General motion detection using GoodFeaturesToTrack and Pyramidal Lucas Kanade
#Author: Arijit Dasgupta

import cv

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

try:
    img = cv.QueryFrame(capture)
except:
    print "Error getting image camera, exiting program"
    exit()

#Initializing images
gray_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
prev_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
eigen_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_32F, 1)
temp_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_32F, 1)
render_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 3)

pyramid1 = cv.CreateImage([img.width + 8, img.height/3], cv.IPL_DEPTH_32F, 1) #Pyramids of Pyramidal Lucas Kanade
pyramid2 = cv.CreateImage([img.width + 8, img.height/3], cv.IPL_DEPTH_32F, 1)

#hardcoded optimizable params, might be implemented with a slider afterwards
quality = 0.1 #cvGoodFeaturesTrack Quality factor
corner_count = 50 #Maximum corners to find with GoodFeaturesToTrack
min_distance = 10 #minimum distance between two corners
skip = 10 #Good Feature Skipper

#Primary initialization
cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
cv.Copy(gray_image, prev_image)
corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track

#Initializing GoodFeatureToTrack execution skip counter
counter = 0

#misc initialization
flag = False #GoodFeatureToTrack execution flag
param1 = 3 #Rotation parameter

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
    #Motion detection
    new_corners, status, track_error = cv.CalcOpticalFlowPyrLK(prev_image, gray_image, pyramid1, pyramid2, corners, (10,10), 2, (cv.CV_TERMCRIT_ITER, 10, 0), 0)
    counter = (counter + 1) % skip
    if(counter == 0):
        corners = cv.GoodFeaturesToTrack(gray_image, eigen_image, temp_image, cornerCount = corner_count, qualityLevel = quality, minDistance = min_distance) #Good features to track
        flag = True
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
                x2, y2 = new_corners[i]
                cv.Line(render_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,255))
                cv.Circle(render_image, (int(x2), int(y2)), 1, (0,255,0))
                sum = sum + (x2 - x1)
                summing_counter = summing_counter + 1
                avg = sum/summing_counter
            except IndexError:
                pass
    #printing movement
    if avg > param1:
        print "Movement right"
    if avg < -param1:
        print "Movement left"
    #Runtime keystroke controls with flags
    key = cv.WaitKey(1)
    if(key == key_quit_lower or key == key_quit_upper):
        cv.DestroyWindow(window1)
        exit()
    elif(key == key_true_image_upper or key == key_true_image_lower):
        flag_true_image = not flag_true_image
    elif(key == key_print_upper or key == key_print_lower):
        print len(new_corners), len(corners), len(status), len(track_error)