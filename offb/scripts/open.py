#!/usr/bin/env python
from __future__ import print_function
from  MY_drone  import  MY_drone
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import mavros_msgs
from cv_bridge import CvBridge, CvBridgeError
import datetime
import zbar
from pyzbar.pyzbar import decode
import numpy as np
class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
    #self.scanner = zbar.Scanner()
    self.error=0
    self.barcode_center=None 
    self.drone=MY_drone("/uav0")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris_0/camera_red_iris/image_raw",Image,self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.scan_qr(cv_image)
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  def scan_qr(self,image):
    
    barcodes = decode(image)
    if barcodes != []:
      self.error+=1
      print("error +1 ")
      if self.drone.FlightMode!='AUTO.LOITER' and self.error>5 :
        self.drone.setHoldMode()
        rospy.loginfo("change mode to Hold Mode ")
        self.error=0
    # loop over the detected barcodes
    for barcode in barcodes:
      # extract the bounding box location of the barcode and draw the
      # bounding box surrounding the barcode on the image
      (x, y, w, h) = barcode.rect
      self.barcode_center=(x+y+w+h)/4
      print(" barcode is in " , barcode.rect)
      cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

      # the barcode data is a bytes object so if we want to draw it on
      # our output image we need to convert it to a string first
      barcodeData = barcode.data.decode("utf-8")
      barcodeType = barcode.type

      # draw the barcode data and barcode type on the image
      text = "{} ({})".format(barcodeData, barcodeType)
      cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 0, 255), 2)


 


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
