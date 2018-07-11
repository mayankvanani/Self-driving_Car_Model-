import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile     
import time
import cv2
import serial
from pyfirmata import util, Arduino


from collections import defaultdict
from io import StringIO
from PIL import Image

sys.path.append("..")
from object_detection.utils import ops as utils_ops
from utils import label_map_util
from utils import visualization_utils as vis_util

## ARDUINO COMMUNICATION AND PIN CONFIGURATION
## define board by the com port (loon in device manager)
board = Arduino('COM12')

# ## start interator for the usage of analog pins
iterator = util.Iterator(board)
iterator.start()

# ## time taken by the arduino to get started
time.sleep(1)
print('..........................................Python ready to communicate with Arduino .............................')


# ## define arduino pins
pwmst = board.get_pin('d:10:p')
drst = board.get_pin('d:8:o')
longIR = board.get_pin('a:1:i')
smallIR = board.get_pin('a:5:i')
pwm1 = board.get_pin('d:3:p')
dr1 = board.get_pin('d:4:o')
pwm2 = board.get_pin('d:6:p')
dr2 = board.get_pin('d:7:o')

## arduino variable
initialVal = 0
val = 0
temp = 0

## small IR sensor value --> cms
# def small_ir():
#     while True:
#         try:
#             small_ir = smallIR.read()
#             y = 17.515649 + ((179.473431) / ((1) + (pow((small_ir * 9.441728531),(2.1360698)))))
#         except Exception as e:
#             continue
#         break
#     return y
    

    

# ## long IR sensor value --> cms
# def long_ir():
#     while True:
#         try:
#             long_ir = longIR.read()
#             y = 82.955628 + ((768.911368) / ((1) + (pow((long_ir * 29.63952791),(6.9436418)))))
#         except Exception as e:
#             continue
#         break
#     return y

## function used to assign pwm for the adaptive cruize control
def funct_value():
    pwm1.write(val)
    dr1.write(0)
    pwm2.write(val)
    dr2.write(0)

def funct_temp():
    pwm1.write(temp)
    dr1.write(0)
    pwm2.write(temp)
    dr2.write(0)





# starting the camera access
## ----------------------------------------------------------------
# cap = cv2.VideoCapture('E:/idealab/turn_2.mp4')
cap = cv2.VideoCapture(1)
## ----------------------------------------------------------------

## defining the ROI points
## -----------------------------------------------------------------
pt1 = [0, 540]  # bottom left 
pt2 = [960, 540] # bottom right
pt3 = [530, 200] # top right
pt4 = [430, 200] # top left
## -----------------------------------------------------------------

def is_detected_object_inside_roi(x,y):
    line_left = y - pt1[1] - (((pt1[1] - pt4[1]) / (pt1[0] - pt4[0])) * (x - pt1[0]))
    line_top = pt3[1]
    line_right = y - pt2[1] - (((pt2[1] - pt3[1]) / (pt2[0] - pt3[0])) * (x - pt2[0]))
    if line_left > 0 and line_top > 0 and line_right > 0:
        return True
    else:
        return False

## distance calibrator
## (x, y) = (estimated_dist in meters, y_pixel_value)
## -----------------------------------------------------------------
initial_pt = (0, 540) ## from the bottom of image
final_pt = (1, 500)   ## to this point in y_direction
## -----------------------------------------------------------------

def distance(x,y):
    aprx_dist = ((y - initial_pt[1]) * (initial_pt[0] - final_pt[0]) / (initial_pt[1] - final_pt[1])) + initial_pt[0]
    return aprx_dist

## Warning!! sign indicator coordinates
sign_coords = (round((pt1[0] + pt2[0]) / 2) , round((pt1[1] + pt4[1]) / 2))

## Y_adj is function to adjust the y axis of the drawn parameters
## ----------------------------------------------------------------
y_adj = 270
## ----------------------------------------------------------------

previous_counter = 479
previous_marker = 479
current_marker = 479
previous_number = 0
FLAG = 1
cnt = 0
countt = 0

def left_marker(imgsrc, x):
    leftmarker = cv2.line(imgsrc, (x, (y_adj - 10)), (x, (y_adj + 10)), (0,0,255), 5)
    return leftmarker

def right_marker(imgsrc, x):
    leftmarker = cv2.line(imgsrc, (x, (y_adj - 10)), (x, (y_adj + 10)), (0,0,255), 5)
    return leftmarker

def steering_marker(imgsrc, x):
    steeringmarker = cv2.line(imgsrc, (x, (y_adj - 10)), (x, (y_adj + 10)), (255,0,0), 5)
    steeringconnector = cv2.line(steeringmarker, (479, y_adj), (x, y_adj), (255,0,0), 2)
    return steeringconnector

def print_number(number):
    font = cv2.FONT_HERSHEY_SIMPLEX
    var_num = cv2.putText(image_np_copy, number, (475, 500), font, 1, (255,255,255), 4, cv2.LINE_AA)
    return var_num

def str_stop():
	pwmst.write(0)
	drst.write(0)

def right_turn(t):
	pwmst.write(1)     ## right
	drst.write(0)
	time.sleep(t)

	pwmst.write(0)     ## stop
	drst.write(0)
	time.sleep(1)

	pwmst.write(1)		## back to neutral
	drst.write(1)
	time.sleep(t)

	pwmst.write(0)		##stop
	drst.write(1)
	time.sleep(1)



def left_turn(t):
	pwmst.write(1)     ## left
	drst.write(1)
	time.sleep(t * 1.04)

	pwmst.write(0)     ## stop
	drst.write(1)
	time.sleep(1)

	pwmst.write(1)		## back to neutral
	drst.write(0)
	time.sleep(t)

	pwmst.write(0)		##stop
	drst.write(0)
	time.sleep(0.5)

def obstacle_avoidance():
    if (x2 < 480):
        pwmst.write(1)
        drst.write(0)
        time.sleep(0.3) 
        pwmst.write(0)
        drst.write(0)
        time.sleep(1)
        pwmst.write(1)
        drst.write(1)
        time.sleep(0.3) 
        pwmst.write(0)
        drst.write(1)
        time.sleep(0.2)

    if (x2 > 480):
        pwmst.write(1)
        drst.write(1)
        time.sleep(0.3)
        pwmst.write(0)
        drst.write(1)
        time.sleep(1)
        pwmst.write(1)
        drst.write(0)
        time.sleep(0.3)
        pwmst.write(0)
        drst.write(0)
        time.sleep(0.2)




# What model to download.
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
# MODEL_FILE = MODEL_NAME + '.tar.gz'
# DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 90


detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)

## ------------------------------------------ main  program -------------------------------------------
## STARTUP LOOP
# 


with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        while True:
            ret, frame = cap.read()
            image_np = cv2.resize(frame, (960,540))
            ## image_np _process for graying, blurring, and various process
            image_np_process = np.copy(image_np)
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            scores = detection_graph.get_tensor_by_name('detection_scores:0')
            classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            # Actual detection.
            (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],
                                                                feed_dict={image_tensor: image_np_expanded})

            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8)


            ##DRAWING LINES on image_np_copy
            image_np_copy = np.copy(image_np) 
            ##left red line
            cv2.line(image_np_copy, (0,y_adj), (300,y_adj), (0,0,255), 1)
            ##right red line
            cv2.line(image_np_copy, (660,y_adj), (960,y_adj), (0,0,255), 1)
            ## centre line
            cv2.line(image_np_copy, (479,(y_adj - 20)), (479,(y_adj +20)), (255,0,0), 5)
            ##proportional band
            cv2.rectangle(image_np_copy, (474,(y_adj - 30)), (484,(y_adj + 30)), (255,255,255), 2 )
            
            ##ROI on image_np_copy
            pts = np.array([pt1, pt2, pt3, pt4], np.int32)
            cv2.polylines(image_np_copy, [pts], True, (255,0,0), 3) 

            ##converting to gray
            gray = cv2.cvtColor(image_np_process, cv2.COLOR_BGR2GRAY)

            ##bluring the gray
            blur_gray = cv2.GaussianBlur(gray, (15,15), 0)

            ## getting px values of drawn line from original image (frame)
            [left_px] = blur_gray[y_adj:(y_adj + 1), 0:300]
            [right_px] = blur_gray[y_adj:(y_adj + 1), 660:960]


            ##                  ----left lane loop----

            left_counter = 300
            for value in reversed(left_px):
                left_counter -= 1
                if (value < 100):
                    break
                else:
                    pass

            left_marker(image_np_copy, left_counter)
            print('left_marker: {}'.format(left_counter))



            ##                  ----right lane loop----

            right_counter = 659
            for value in (right_px):
                right_counter += 1
                if (value < 100):
                    break
                else:
                    pass
            right_marker(image_np_copy, right_counter)
            print('right_marker: {}'.format(right_counter))

            ##                ----- steering logic-----

            ##averaging
            steering_counter = np.int32((left_counter + right_counter)/2)
            print('steering_counter: {}'.format(steering_counter))

            decision = steering_counter - previous_counter
            print('decision: {}'.format(decision))
            if (abs(decision) < 30):
                # current_marker = np.int32(479 + decision)
                print('iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii')
                ## positional conditioning and DEAD BAND
                if (steering_counter >= 474 and steering_counter <= 484):
                    steering_counter = 479
                    current_marker = 479
                    decision = 0
                    print_number('0')
                    current_number = 0
                    steering_marker(image_np_copy, current_marker)
                    
                
                elif (steering_counter >= 427 and steering_counter < 474):
                    current_marker = 469
                    print_number('-1')
                    current_number = -1
                    steering_marker(image_np_copy, current_marker)

                    
                elif (steering_counter >= 380 and steering_counter < 427):
                    current_marker = 459
                    print_number('-2')
                    current_number = -2
                    steering_marker(image_np_copy, current_marker)

                    
                elif (steering_counter < 380):
                    current_marker = 449
                    print_number('-3')
                    current_number = -3
                    steering_marker(image_np_copy, current_marker)

                    
                elif (steering_counter > 484 and steering_counter <= 514):
                    current_marker = 489
                    print_number('1')
                    current_number = 1
                    steering_marker(image_np_copy, current_marker)

                    
                elif (steering_counter > 514 and steering_counter <= 544):
                    current_marker = 499
                    print_number('2')
                    current_number = 2
                    steering_marker(image_np_copy, current_marker)

                    
                elif (steering_counter > 544):
                    current_marker = 509
                    print_number('3')
                    current_number = 3
                    steering_marker(image_np_copy, current_marker)


            else:
                print('eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
                current_number = previous_number
                print_number(str(previous_number))      
                steering_marker(image_np_copy, previous_marker)
                previous_counter = steering_counter


            previous_number = current_number
            previous_marker = current_marker
            previous_counter = steering_counter
            print('previous_number: {}'.format(previous_number))
            print('current_marker: {}'. format(current_marker))
 


            for i,b in enumerate(boxes[0]):
                # if (classes[0][i] == 1):
                if scores[0][i] > 0.6:
                    # print('classes: {}'.format(classes[0][i]))
                    # print('bounding box: {}'.format(boxes[0][i]))
                    [y1,x1,y2,x2] = boxes[0][i]
                    x1 = round(960 * x1).astype(np.uint32)
                    x2 = round(960 * x2).astype(np.uint32)
                    y1 = round(540 * y1).astype(np.uint32)
                    y2 = round(540 * y2).astype(np.uint32)
                    print([x1, y1, x2, y2])
                    aprx_dist = distance(x2,y2)
                    cv2.putText(image_np_copy, '{}m'.format(aprx_dist), (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                    FLAG = 1
                    if (is_detected_object_inside_roi(x1,y1) or is_detected_object_inside_roi(x2,y2)) == True:
                        FLAG = 0
                        if (y2 >= 426):
                            cv2.putText(image_np_copy, 'COLLISION WARNING!!!!', sign_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                            val = 0
                            funct_value()
                            pwmst.write(0)
                            drst.write(0)
                            # if (abs(val - temp) < 0.5):
                            #     funct_value()
                            # else:
                            #     funct_temp()
                        elif (y2 < 426 and y2 > 313):
                            cv2.putText(image_np_copy, 'WARNING!!!!', sign_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                            val = 0
                            funct_value()
                            pwmst.write(0)
                            drst.write(0)
                            time.sleep(0.2)
                            # obstacle_avoidance()
                            # if (abs(val - temp) < 0.5):
                            #     funct_value()
                            # else:
                            #     funct_temp()
                        elif(y2 <= 313 and y2 >= 200):
                            cv2.putText(image_np_copy, 'OBJECT IN VICINITY!!!!', sign_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                            val = 0
                            funct_value()
                            pwmst.write(0)
                            drst.write(0)
                            time.sleep(0.2)

                            # if (abs(val - temp) < 0.5):
                            #     funct_value()
                            # else:
                            #     funct_temp()
                            # obstacle_avoidance()

                            


                        # temp = val
                    else:
                        FLAG = 1

             ## steering arduino control
            if (FLAG == 1):    
                if (countt==0):
                    val = 0.2
                    funct_value()
                    time.sleep(0.5)
                    val = 0.3
                    funct_value()
                    time.sleep(0.5)
                    val = 0.4
                    funct_value()
                    time.sleep(0.5)
                    val = 0.5
                    funct_value()
                    time.sleep(0.5)
                else:   
                    val = 0.5 
                    funct_value()

                if(current_number == 1):
                    right_turn(0.1)
                elif(current_number == 2):  
                    right_turn(0.2)
                elif(current_number == 3):
                    right_turn(0.3)
                elif(current_number == -1):
                    left_turn(0.1)
                elif(current_number == -2):
                    left_turn(0.2)
                elif(current_number == -3):
                    left_turn(0.3)
                elif(current_number == 0):
                    pwmst.write(0)
                    drst.write(0)
                    time.sleep(0.5)



            else:
                FLAG  = 1
 



            countt += 1
            print('----------------------------------------------------------------------------------')
            # time.sleep(0.2)
            cv2.imshow('image_np', image_np)
            cv2.imshow('image_np_copy', image_np_copy)
            if cv2.waitKey(1) & 0xFF == ord('q'):
              cv2.destroyAllWindows()
              cap.release()
              board.exit()
              time.sleep(2)
              break
 
