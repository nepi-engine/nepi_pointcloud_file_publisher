#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
import numpy as np
import cv2
import open3d as o3d
import random



from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img


from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_img 

rom nepi_ros_interfaces.msg import FilePublisherStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################
# Node Class
#########################################

class NepiFilePublisherApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"
  PUBLISH_TOPIC_PREFIX = "file_"

  IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']
  VID_FILE_TYPES = ['avi','AVI']


  #Set Initial Values
  MIN_IMG_DELAY = 0.05
  MAX_IMG_DELAY = 5.0
  FACTORY_IMG_PUB_DELAY = 1.0
  STANDARD_IMAGE_SIZES = ['630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']
  FACTORY_IMG_SIZE = '630 x 900'
  IMG_PUB_ENCODING_OPTIONS = ["bgr8","rgb8","mono8"]
  FACTORY_IMG_ENCODING_OPTION = "bgr8" 

  UPDATER_DELAY_SEC = 1.0
  
  
  paused = False
  last_folder = ""
  current_folders = []

  img_running = False
  img_count = 0
  img_pub = None


  vid_running = False
  vid_count = 0
  vid_pub = None

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "file_publisher_app" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    

    ## App Setup ########################################################
    self.initParamServerValues(do_updates=False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)

    # Create class publishers
    self.status_pub = rospy.Publisher("~status", FilePublisherStatus, queue_size=1, latch=True)

    # Start updater process
    rospy.Timer(rospy.Duration(self.UPDATER_DELAY_SEC), self.updaterCb())

    # General Class Subscribers
    rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    rospy.Subscriber('~set_folder', String, self.setFolderCb)
    rospy.Subscriber('~pause_pub', Bool, self.pausePubCb)

    # Image Pub Scubscirbers and publishers
    rospy.Subscriber('~img_pub_size', String, self.imgPubSizeCb)
    rospy.Subscriber('~img_pub_encoding', String, self.imgPubEncodingCb)
    rospy.Subscriber('~set_img_pub_random', Bool, self.setImgPubRandomCb)
    rospy.Subscriber('~set_img_pub_delay', Float32, self.setImgPubDelayCb) 
    rospy.Subscriber('~start_img_pub', Empty, self.startImgPubCb)
    rospy.Subscriber('~stop_img_pub', Empty, self.stopImgPubCb)

    # Video Pub Scubscirbers and publishers
    rospy.Subscriber('~vid_pub_size', String, self.vidPubSizeCb)
    rospy.Subscriber('~vid_pub_encoding', String, self.vidPubEncodingCb)
    rospy.Subscriber('~start_vid_pub', Empty, self.startVidPubCb)
    rospy.Subscriber('~stop_vid_pub', Empty, self.stopVidPubCb)
    rospy.Subscriber('~set_vid_pub_random', Bool, self.setVidPubRandomCb)

    time.sleep(1)

    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    self.publish_status()
    # Spin forever (until object is detected)
    rospy.spin()


  #############################
  ## APP callbacks

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = rospy.get_param('~current_folder', self.init_current_folder)

    # Update folder info
    if current_folder != self.last_folder:
      update_status = True
      if os.path.exists(current_folder):

        self.current_folders = nepi_ros.get_folder_list(current_folder)

        num_files = 0
        for f_type in IMG_FILE_TYPES:
          num_files = num_files + nepi_ros.get_file_count(current_folder,f_type)
        self.img_count =  num_files

        num_files = 0
        for f_type in VID_FILE_TYPES:
          num_files = num_files + nepi_ros.get_file_count(current_folder,f_type)
        self.vid_count =  num_files


    self.last_folder = current_folder

    if update_status == True:
      self.publish_status()

  def setFolderCb(self,msg):
    new_folder = msg.data
    if os.path.exists(new_folder):
      rospy.set_param('~current_folder',new_folder)
    self.publish_status()


  def pausePubCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    self.paused = msg.data
    self.publish_status()


  #############################
  ## Image callbacks

  def imgPubSizeCb(self,msg):
    new_size = msg.data
    if new_size in self.STANDARD_IMAGE_SIZES:
      rospy.get_param('~img_size',new_size)
    self.publish_status()

  def imgPubEncodingCb(self,msg):
    new_encoding = msg.data
    if new_encoding in self.IMG_PUB_ENCODING_OPTIONS:
      rospy.get_param('~img_encoding',new_encoding)
    self.publish_status()

  def imgPubRandomCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    rospy.set_param('~img_random',msg.data)
    self.publish_status()

  def imgPubDelayCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    delay = msg.data
    if delay < self.MIN_IMG_DELAY:
      delay = self.MIN_IMG_DELAY
    if delay > self.MAX_IMG_DELAY:
      delay = self.MAX_IMG_DELAY
    rospy.set_param('~img_delay',delay)
    self.publish_status()


  def startImgPubCb(self):
    if self.img_running == False:
      current_folder = rospy.get_param('~current_folder', self.init_current_folder)
      if self.img_pub != None
        self.img_pub.unregister()
        sleep(1)
      self.image_pub = rospy.Publisher("~images", Image, queue_size=1, latch=True)
      # Now start publishing images
      self.img_file_list = []
      self.img_num_files = 0
      if os.path.exists(current_folder):
        for f_type in self.IMG_FILE_TYPES:
          [file_list, num_files] = nepi_ros.get_file_list(current_folder,f_type)
          self.img_file_list.extend(file_list)
          self.img_num_files += num_files
        if self.img_num_files > 0:
          self.current_img_ind = 0
          rospy.Timer(rospy.Duration(1), self.imgPubCb, oneshot = True)
          self.img_running = True
          rospy.set_param('~img_running',True)
        else:
          print("No image files found in folder " + current_folder + " not found")
      else:
        print("Folder " + current_folder + " not found")

  def stopImgPubCb(self):
    self.img_running = False
    rospy.set_param('~img_running',False)
    sleep(1)
    if self.img_pub != None
      self.img_pub.unregister()
      sleep(1)


  ### Add your CV2 image customization code here
  def imgPubCb(self,timer):
    img_size = rospy.get_param('~img_size',self.init_img_size)
    img_encoding = rospy.get_param('~img_encoding',self.init_img_encoding)
    img_random = rospy.get_param('~img_random',self.init_img_random)
    img_overlay = rospy.get_param('~img_overlay',  self.init_img_overlay)
    if self.img_running == True and self.paused == False:
      if self.img_pub != None:
        if img_random == True:
          self.current_img_ind = int(random.random() * self.img_num_files)
        if self.current_img_ind > (self.num_files-1):
          self.current_img_ind = 0 # Start over
        file2open = self.file_list[self.current_img_ind]
        self.current_img_ind = self.current_img_ind + 1
        #print("Opening File: " + file2open)
        cv_image = cv2.imread(file2open)
        shape = cv_image.shape
        #print(shape)
        try:
          img_size_list = img_size.split(":")
          img_h = int(img_size_list[0])
          img_w = int(img_size_list[1])
        except:
          img_h = 600
          img_w = 800
        cv_image = cv2.resize(cv_image,(img_h,img_w))
        #Convert image from cv2 to ros
        img_out_msg = nepi_img.cv2img_to_rosimg(cv_image,encoding=img_encoding)
        # Publish new image to ros
        if not rospy.is_shutdown():
          img_out_msg.header.stamp = rospy.Time.now()
          self.img_pub.publish(img_out_msg) 
    if self.img_running == True:
      img_delay = rospy.get_param('~img_delay',  self.init_img_delay) -1
      if img_delay < 0:
        img_delay == 0
      nepi_ros.sleep(img_delay)
      rospy.Timer(rospy.Duration(1), self.imgPubCb, oneshot = True)


  #############################
  ## Video callbacks

  def vidPubSizeCb(self,msg):
    new_size = msg.data
    if new_size in self.STANDARD_IMAGE_SIZES:
      rospy.get_param('~vid_size',new_size)
    self.publish_status()

  def vidPubEncodingCb(self,msg):
    new_encoding = msg.data
    if new_encoding in self.IMG_PUB_ENCODING_OPTIONS:
      rospy.get_param('~vid_encoding',new_encoding)
    self.publish_status()

  def vidPubRandomCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    rospy.set_param('~vid_random',msg.data)
    self.publish_status()


  def startVidPubCb(self):
    if self.vid_running == False:
      current_folder = rospy.get_param('~current_folder', self.init_current_folder)
      if self.vid_pub != None
        self.vid_pub.unregister()
        sleep(1)
      self.image_pub = rospy.Publisher("~images", Image, queue_size=1, latch=True)
      # Now start publishing images
      self.vid_file_list = []
      self.vid_num_files = 0
      if os.path.exists(current_folder):
        for f_type in self.VID_FILE_TYPES:
          [file_list, num_files] = nepi_ros.get_file_list(current_folder,f_type)
          self.vid_file_list.extend(file_list)
          self.vid_num_files += num_files
        if self.vid_num_files > 0:
          self.current_vid_ind = 0
          rospy.Timer(rospy.Duration(1), self.vidPubCb, oneshot = True)
          self.vid_running = True
          rospy.set_param('~vid_running',True)
        else:
          print("No image files found in folder " + current_folder + " not found")
      else:
        print("Folder " + current_folder + " not found")

  def stopVidPubCb(self):
    self.vid_running = False
    rospy.set_param('~vid_running',False)
    sleep(1)
    if self.vid_pub != None
      self.vid_pub.unregister()
      sleep(1)


  ### Add your CV2 image customization code here
  def vidPubCb(self,timer):
    vid_size = rospy.get_param('~vid_size',self.init_vid_size)
    vid_encoding = rospy.get_param('~vid_encoding',self.init_vid_encoding)
    vid_random = rospy.get_param('~vid_random',self.init_vid_random)
    vid_overlay = rospy.get_param('~vid_overlay',  self.init_vid_overlay)
    if self.vid_running == True and self.paused == False:
      if self.vid_pub != None:
        if vid_random == True:
          self.current_vid_ind = int(random.random() * self.vid_num_files)
        if self.current_vid_ind > (self.num_files-1):
          self.current_vid_ind = 0 # Start over
        file2open = self.file_list[self.current_vid_ind]
        self.current_vid_ind = self.current_vid_ind + 1
        #print("Opening File: " + file2open)


        if os.path.isfile(self.file2open):
          print("Opening File: " + self.file2open)
          self.vidcap = cv2.VideoCapture(self.file2open)
          if self.vidcap.isOpened() == True:
            success,image = self.vidcap.read()
            shape_str = str(image.shape)
            print('Image size: ' + shape_str)
            fps = self.vidcap.get(5)
            print('Frames per second : ', fps,'FPS')

            frame_count = self.vidcap.get(7)
            print('Frame count : ', frame_count)

            while success == True and self.vid_running == True and not rospy.is_shutdown():
              if self.paused == True:
                time.sleep(1)
              else:
                # Publish video at native fps
                success,cv_image = self.vidcap.read()
                if success == False:
                  self.vidcap.release()
                  time.sleep(1)
                  self.vidcap = cv2.VideoCapture(self.file2open)
                else: 
                  try:
                    vid_size_list = vid_size.split(":")
                    vid_h = int(vid_size_list[0])
                    vid_w = int(vid_size_list[1])
                  except:
                    vid_h = 600
                    vid_w = 800
                  cv_image = cv2.resize(cv_image,(vid_h,vid_w))
                  #Convert image from cv2 to ros
                  img_out_msg = nepi_img.cv2img_to_rosimg(cv_image,encoding=vid_encoding)
                  # Publish new image to ros
                  if not rospy.is_shutdown():
                    img_out_msg.header.stamp = rospy.Time.now()
                    self.vid_pub.publish(img_out_msg) 

          else:
            print("Unable to grap image from video file")
            return
        else:
          print("File not found in specified folder")
          return
    if self.vid_running == True:
      rospy.Timer(rospy.Duration(1), self.vidPubCb, oneshot = True)


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = FilePublisherStatus()

    status_msg.home_folder = self.HOME_FOLDER
    status_msg.current_folder = rospy.get_param('~current_folder', self.init_current_folder)
    status_msg.current_folders = self.current_folders

    status_msg.paused = self.paused

    status_msg.img_size_options_list = self.STANDARD_IMAGE_SIZES
    status_msg.img_pub_size = rospy.get_param('~img_size',self.init_img_size)
    status_msg.img_encoding_options_list = self.IMG_PUB_ENCODING_OPTIONS
    status_msg.img_pub_encoding = rospy.get_param('~img_encoding',self.init_img_encoding)


    status_msg.img_count = self.img_count
    status_msg.img_random = rospy.get_param('~img_random',self.init_img_random)
    status_msg.img_overlay = rospy.get_param('~img_overlay',  self.init_img_overlay)
    status_msg.min_max_img_delay = [self.MIN_IMG_DELAY. self.MAX_IMG_DELAY]
    status_msg.img_delay = rospy.get_param('~img_delay',  self.init_img_delay)
    status_msg.img_pub_running = rospy.get_param('~img_running',self.init_img_running)


    status_msg.vid_size_options_list = self.STANDARD_IMAGE_SIZES
    status_msg.vid_pub_size = rospy.get_param('~vid_size',self.init_vid_size)
    status_msg.vid_encoding_options_list = self.IMG_PUB_ENCODING_OPTIONS
    status_msg.vid_pub_encoding = rospy.get_param('~vid_encoding',self.init_vid_encoding)

    status_msg.vid_count = self.vid_count
    status_msg.vid_random = rospy.get_param('~vid_random',self.init_vid_random)
    status_msg.vid_overlay = rospy.get_param('~vid_overlay',self.init_vid_overlay)
    status_msg.vid_pub_running =  rospy.get_param('~vid_running', self.init_vid_running)

    self.status_pub.publish(status_msg)





  #######################
  ### App Config Functions

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~current_folder', self.HOME_FOLDER)

    rospy.get_param('~img_size',self.FACTORY_IMG_SIZE)
    rospy.get_param('~img_encoding',self.FACTORY_IMG_ENCODING_OPTION)

    rospy.get_param('~img_size',self.init_img_size)
    rospy.get_param('~img_encoding',self.init_img_encoding)

    rospy.set_param('~img_random',False)
    rospy.set_param('~img_overaly',False)
    rospy.set_param('~img_delay', self.FACTORY_IMG_PUB_DELAY)
    rospy.set_param('~img_running', False)

    rospy.get_param('~vid_size',self.FACTORY_IMG_SIZE)
    rospy.get_param('~vid_encoding',self.FACTORY_IMG_ENCODING_OPTION)

    rospy.set_param('~vid_random',False)
    rospy.set_param('~vid_overlay',False)
    rospy.set_param('~vid_running', False)

    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
    self.init_current_folder = rospy.get_param('~current_folder', self.HOME_FOLDER)

    self.init_img_size = rospy.get_param('~img_size',self.FACTORY_IMG_SIZE)
    self.init_img_encoding = rospy.get_param('~img_encoding',self.FACTORY_IMG_ENCODING_OPTION)

    self.init_img_random = rospy.get_param('~img_random',False)
    self.init_img_overlay = rospy.get_param('~img_overlay',False)
    self.init_img_delay = rospy.get_param('~img_delay', self.FACTORY_IMG_PUB_DELAY)
    self.init_img_running = rospy.get_param('~img_running', False)

    self.init_vid_size = rospy.get_param('~vid_size',self.FACTORY_IMG_SIZE)
    self.init_vid_encoding = rospy.get_param('~vid_encoding',self.FACTORY_IMG_ENCODING_OPTION)

    self.init_vid_random = rospy.get_param('~vid_random',False)
    self.init_vid_overlay = rospy.get_param('~vid_overlay',False)
    self.init_vid_running = rospy.get_param('~vid_running', False)

    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    rospy.set_param('~current_folder', self.init_current_folder)

    rospy.set_param('~img_size',self.init_img_size)
    rospy.set_param('~img_encoding',self.init_img_encoding)

    rospy.set_param('~img_random',self.init_img_random)
    rospy.set_param('~img_overlay',  self.init_img_overlay)
    rospy.set_param('~img_delay',  self.init_img_delay)
    rospy.set_param('~img_running',self.init_img_running)


    rospy.set_param('~vid_size',self.init_vid_size)
    rospy.set_param('~vid_encoding',self.init_vid_encoding)

    rospy.set_param('~vid_random',self.init_vid_random)
    rospy.set_param('~vid_overlay',self.init_vid_overlay)
    rospy.set_param('~vid_running', self.init_vid_running)

    if do_updates:
      self.updateFromParamServer()
      self.publish_status()


               
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self," Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePublisherApp()







