#!/usr/bin/env python3
import queue
import cv2
import time
import imutils
import argparse
import numpy as np

from imutils.video import FPS
from imutils.video import VideoStream

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

from object_msgs.msg import Object as obj
from object_msgs.msg import Objects as objs
from object_msgs.msg import ObjectInBox
from object_msgs.msg import ObjectsInBoxes


class ObjectDetection:
    def __init__(self):
        self.imageSub = rospy.Subscriber("/camera/image_raw",Image,self.imageCallback, queue_size = 1) # default /camera/color/image_raw
        self.imagePub = rospy.Publisher("/detections/compressed",CompressedImage, queue_size = 1) 
        self.objectPub = rospy.Publisher("object_detection/objects_msg", ObjectsInBoxes, queue_size = 1)
        param_name = rospy.search_param('img_width')
        self.img_width = rospy.get_param(param_name)
        param_name = rospy.search_param('img_height')
        self.img_height = rospy.get_param(param_name)
        param_name = rospy.search_param('confidence')
        self.confidence_threshold = rospy.get_param(param_name)
        rospy.set_param('/object_detection_img/compressed/jpeg_quality', 20)
        #-------------------------------------------------------------##
        #Initialize Objects and corresponding colors which the model can detect
        self.labels = ["background", "aeroplane", "bicycle", "bird", 
                       "boat","bottle", "bus", "car", "cat", "chair", "cow", 
                       "diningtable","dog", "horse", "motorbike", "person", "pottedplant", 
                       "sheep","sofa", "train", "tvmonitor"]
        self.colors = np.random.uniform(0, 255, size=(len(self.labels), 3))
        self.fps = FPS().start()

        #Loading Caffe Model
        print("[Status] Loading Model")
        path_models = '/home/yakir/customers_ws/src/customer_sim_patrol/object_detection/src/Caffe/'
        self.nn = cv2.dnn.readNetFromCaffe(path_models+'SSD_MobileNet_prototxt.txt', path_models+'SSD_MobileNet.caffemodel')
        time.sleep(2.0)
        print('[Status] Starting Video Stream...')


    def imageCallback(self,rgb_msg):
        bgr_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        self.runDetector(bgr_image)
    
    def runDetector(self, image):
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)        
        #Resize Frame to 400 pixels
        image = imutils.resize(image, width=400)
        (h, w) = image.shape[:2]

        #Converting Frame to Blob
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (self.img_width, self.img_height)), 0.007843, (self.img_width, self.img_height), 127.5)

        #Passing Blob through network to detect and predict
        self.nn.setInput(blob)
        detections = self.nn.forward()

        objs_in = ObjectsInBoxes()

        #Loop over the detections
        for i in np.arange(0, detections.shape[2]):
        
        #Extracting the confidence of predictions
            confidence = detections[0, 0, i, 2]

            #Filtering out weak predictions
            if confidence > self.confidence_threshold:
                
                object = obj()
                obj_in = ObjectInBox()

                #Extracting the index of the labels from the detection
                #Computing the (x,y) - coordinates of the bounding box        
                idx = int(detections[0, 0, i, 1])

                #Extracting bounding box coordinates
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                #Drawing the prediction and bounding box
                label = "{}: {:.2f}%".format(self.labels[idx], confidence * 100)
                cv2.rectangle(image, (startX, startY), (endX, endY), self.colors[idx], 2)

                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors[idx], 2)

                # add object to the array
                object.object_name = self.labels[idx]
                object.probability = confidence
                obj_in.object = object
                obj_in.roi.x_offset = np.uint32(np.int32(startX))
                obj_in.roi.y_offset = np.uint32(np.int32(startY))
                obj_in.roi.height = np.uint32(np.int32(endY - startY))
                obj_in.roi.width = np.uint32(np.int32(endX - startX))
                objs_in.objects_vector.append(obj_in)
                

                

        # publish the img
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()  
        self.imagePub.publish(msg)
        self.fps.update()      

        #publish the b-box array 
        objs_in.header.stamp = rospy.Time.now()
        self.objectPub.publish(objs_in)

    def shutdown_hook(self):
        self.fps.stop()
        print("[Info] Elapsed time: {:.2f}".format(self.fps.elapsed()))
        print("[Info] Approximate FPS: {:.2f}".format(self.fps.fps()))
        

#ap.add_argument("-c", "--confidence", type = float, default = 0.7)

if __name__ == '__main__':
    rospy.init_node('object_detection_node', anonymous=True)
    print('Success')
    object_detection = ObjectDetection()
    rospy.on_shutdown(object_detection.shutdown_hook)

    rospy.spin()
