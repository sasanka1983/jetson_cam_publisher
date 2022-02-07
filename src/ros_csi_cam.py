#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import sys
import cv2
from jetcam.csi_camera import CSICamera
import getpass
import os

class ros_csi_cam:
    def __init__(self,width,height,capture_width,capture_height,capture_fps):
        
        self.width=width
        self.height=height
        self.capture_width=capture_width
        self.capture_height=capture_height
        self.capture_fps=capture_fps
    
    def gstreamer_pipeline(self,
    sensor_id=1,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
        return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

    def capture_image(self):
        #cam = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0))
        cam = CSICamera(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
        if cam.isOpened():

            rospy.init_node("jetson_csi_camera",anonymous=True)
            sleep_rate = rospy.Rate(.1)
            img_pub=rospy.Publisher("csi_img",Image,queue_size=20)   
            bridge=CvBridge() 
            image_count=1
            while not rospy.is_shutdown():
                    
                    try:
                        image= cam.read()  
                        
                        img = bridge.cv2_to_imgmsg(image,'bgr8')
                        img_pub.publish(img)
                        print("published image {imagecount}" .format(imagecount=image_count) )   
                        image_count=image_count+1
                        sleep_rate.sleep()
                        #cv2.imshow('frame',image)
                        #cv2.waitKey(10)
                    except(CvBridgeError):
                        print("unable to convert opencv image to ros image")
                        #cv2.destroyAllWindows()
                        cam.release()
                        break
                    
                    except(KeyboardInterrupt):
                        print("stopping camera")
                        #cv2.destroyAllWindows()
                        cam.release()
                        break   
        else:
            print("unable to capture video from CSI cam")
    print("Camera released. Stopping the node")

if __name__=='__main__':
    print("Capturing images from CSI camera")
    #password = getpass.getpass()
    #command = 'sudo -S systemctl restart nvargus-daemon'
    #os.system('echo %s | %s' % (password, command))
    if(len(sys.argv)==1):
        
        print("capturing image with width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30")
        this_obj=ros_csi_cam(width=224, height=224, capture_width=1080, capture_height=720, capture_fps=30)
    elif(len(sys.argv)==5):
        print("capturing image with width=, height=480, capture_width=1080, capture_height=720, capture_fps=30")
        this_obj=ros_csi_cam(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5])
    else:
        print("please provide arguments for width, height, capture_width, capture_height, capture_fps in the same order")  
        exit()  
    this_obj.capture_image()