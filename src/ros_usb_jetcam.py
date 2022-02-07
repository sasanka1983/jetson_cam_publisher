#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import sys
from jetcam.usb_camera import USBCamera

class ros_usb_jetcam:
    def __init__(self,video_port,width,height,capture_width,capture_height,capture_fps):
        self.video_port=video_port
        self.width=width
        self.height=height
        self.capture_width=capture_width
        self.capture_height=capture_height
        self.capture_fps=capture_fps
       

    def capture_image(self):
        cam= USBCamera(self.video_port,self.width, self.height, self.capture_width, self.capture_height, self.capture_fps)
        
        
        rospy.init_node("jetson_usb_jetcamera",anonymous=True)
        sleep_rate = rospy.Rate(.1)
        img_pub=rospy.Publisher("jetson_usb_jetcamera_image",Image,queue_size=10)   
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
                    cv2.imshow('frame',image)
                    cv2.waitKey(10)
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
        
    print("Camera released. Stopping the node")

if __name__=='__main__':
    print("Capturing images from USB Jet camera")
    if(len(sys.argv)==1):
        
        print("capturing image with width=640, height=480, capture_width=1080, capture_height=720, capture_fps=30")
        this_obj=ros_usb_jetcam("1",width=640, height=480, capture_width=1080, capture_height=720, capture_fps=30)
    elif(len(sys.argv)==6):
        print("capturing image with device={arg1}, width={arg2}, height={arg3}, capture_width={arg4}, capture_height={arg5}, capture_fps={arg6}" .format(arg1=sys.argv[1],arg2=sys.argv[2],arg3=sys.argv[3],arg4=sys.argv[4],arg5=sys.argv[5],arg6=sys.argv[6]))
        this_obj=ros_usb_jetcam(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6])
    else:
        print("please provide arguments for DEVICE_ID, width, height, capture_width, capture_height, capture_fps in the same order")  
        exit()  
    this_obj.capture_image()