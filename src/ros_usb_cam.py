#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import sys

class ros_usb_cam:
    def __init__(self,video_port):
        self.video_port=video_port

    def capture_image(self):
        cam= cv2.VideoCapture(self.video_port)
        
        if(cam.isOpened()==False):
            print("Unable to open camera at {videoport}" .format(videoport=self.video_port))
        else:
            rospy.init_node("jetson_usb_camera",anonymous=True)
            sleep_rate = rospy.Rate(.1)
            img_pub=rospy.Publisher("usb_img",Image,queue_size=10)   
            bridge=CvBridge() 
            image_count=1
            while not rospy.is_shutdown() and cam.isOpened()==True:
                
                try:
                    check,frame= cam.read()  
                    if(check==False):
                        print("capturing image failed")
                    else:
                        sleep_rate.sleep()
                        img = bridge.cv2_to_imgmsg(frame,'bgr8')
                        img_pub.publish(img)
                        print("published image {imagecount}" .format(imagecount=image_count) )   
                        image_count=image_count+1
                        #cv2.imshow('frame',frame)
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
        
        print("Camera released. Stopping the node")

if __name__=='__main__':
    print("Capturing images from USB camera at port {port}" .format(port=sys.argv[1]))
    this_obj=ros_usb_cam(sys.argv[1])
    this_obj.capture_image()