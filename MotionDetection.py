#Universal motion detection algorithm implementation
#Current version: 1D X-coord movement detection...
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
accumulator = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
gray_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
temp_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
temp_image2 = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
sum_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 1)
render_image = cv.CreateImage([img.width, img.height], cv.IPL_DEPTH_8U, 3)

#Memory storage for contour detection
store = cv.CreateMemStorage()

#hardcoded optimizable params, might be implemented with a slider afterwards
rotation_threshold = 5 #Rotation comparison threshold
minimum_contour_area_for_movement = 400.0 #Minimum contour area for movement positive
upperLimit2 = cv.Scalar(255)
lowerLimit2 = cv.Scalar(200) #2nd stage image thresholds, dilated additive difference image
upperLimit1 = cv.Scalar(40)
lowerLimit1 = cv.Scalar(30) #1st stage image thresholds, difference image
filter_depth = 15 #Low passs moving avg. filter depth

#rotation parameter store
param1 = 0

#rotation filter register initialization
filter_register = []
for i in range(filter_depth):
    filter_register.append(0)
    
#Primary image fetching
cv.CvtColor(img, gray_image, cv.CV_RGB2GRAY)
cv.Copy(gray_image, temp_image)

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
    cv.Smooth(gray_image, gray_image)
    cv.AbsDiff(gray_image, temp_image, temp_image2)
    cv.Copy(temp_image2, accumulator)
    cv.InRangeS(accumulator, lowerLimit1, upperLimit1, accumulator) #Difference image thresholding
    cv.Add(accumulator, sum_image, sum_image, accumulator)
    cv.SubS(sum_image,10,sum_image)
    cv.Copy(sum_image, accumulator)
    cv.Dilate(accumulator, accumulator, None, 2)
    cv.InRangeS(accumulator, lowerLimit2, upperLimit2, accumulator) #Final thresholding
    cv.Copy(accumulator, temp_image2)
    contours = cv.FindContours(temp_image2, store, cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_NONE)
    cv.CvtColor(accumulator, render_image, cv.CV_GRAY2RGB)
    cv.Copy(gray_image, temp_image)
    #Motion detection
    max_contour_area = minimum_contour_area_for_movement
    largest_contour = []
    if len(contours) != 0:
        avg = param1
        temp_contour = contours
        while temp_contour != None: #Loop to get the largest contour
            contour_area = cv.ContourArea(temp_contour)
            if(contour_area > max_contour_area):
                max_contour_area = contour_area
                largest_contour = temp_contour
            temp_contour = temp_contour.h_next()
        if largest_contour != []: #Getting the average x-coords of the largest contour
            cv.DrawContours(render_image, largest_contour, (0,255,0), (255,0,0), 1, 2) #Drawing only the largest contour
            rect = cv.BoundingRect(largest_contour)
            cv.Rectangle(render_image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0,0,255), 1) #Drawing rectangle for movement...
            sum = 0
            for i in largest_contour:
                sum = sum + i[0]
            avg = sum/len(largest_contour)
        #Adding a moving average filter to the average output
        sum = 0
        filtered = 0
        filter_register.pop(0)
        filter_register.append(avg)
        for i in filter_register:
                sum = sum + i
        filtered = sum/filter_depth
        avg = filtered
        #Rotation output
        if param1 - avg > rotation_threshold:
            print 'movement left'
        elif param1 - avg < -rotation_threshold:
            print 'movement right'
        param1 = avg
    #Runtime keystroke controls with flags
    key = cv.WaitKey(1)
    if(key == key_quit_lower or key == key_quit_upper):
        cv.DestroyWindow(window1)
        exit()
    elif(key == key_true_image_upper or key == key_true_image_lower):
        flag_true_image = not flag_true_image
    elif(key == key_print_upper or key == key_print_lower):
        print "Nothing to print"