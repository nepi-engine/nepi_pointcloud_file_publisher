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
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
import numpy as np
import cv2
import open3d as o3d



from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img


from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_img 

from nepi_app_file_pub_img.msg import FilePubImgStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################
# Node Class
#########################################

class NepiFilePubImgApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['pcd']

  #Set Initial Values
  MIN_DELAY = 0.05
  MAX_DELAY = 5.0
  FACTORY_IMG_PUB_DELAY = 1.0

  UPDATER_DELAY_SEC = 1.0
  

  last_folder = ""
  current_folders = []
  current_file_list = []
  current_topic_list = []

  running = False
  file_count = 0
  pub_pub = None


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_img" # Can be overwitten by luanch command
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
    self.status_pub = rospy.Publisher("~status", FilePubImgStatus, queue_size=1, latch=True)

    # Start updater process
    rospy.Timer(rospy.Duration(self.UPDATER_DELAY_SEC), self.updaterCb)

    # General Class Subscribers
    rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    rospy.Subscriber('~select_folder', String, self.selectFolderCb)
    rospy.Subscriber('~home_folder', Empty, self.homeFolderCb)
    rospy.Subscriber('~back_folder', Empty, self.backFolderCb)

    # Image Pub Scubscirbers and publishers
    rospy.Subscriber('~set_delay', Float32, self.setDelayCb) 
    rospy.Subscriber('~start_pub', Empty, self.startPubCb)
    rospy.Subscriber('~stop_pub', Empty, self.stopPubCb)

    rospy.Subscriber('~pause_pub', Bool, self.pausePubCb)

    time.sleep(1)

    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    self.publish_status()
    # Spin forever (until object is detected)
    rospy.spin()

  #######################
  ### App Config Functions

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~current_folder', self.HOME_FOLDER)

    rospy.set_param('~delay', self.FACTORY_IMG_PUB_DELAY)

    rospy.set_param('~pub_transforms',  False)
    rospy.set_param('~create_transforms', False)

    rospy.set_param('~running', False)

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

    self.init_delay = rospy.get_param('~delay', self.FACTORY_IMG_PUB_DELAY)
    self.init_running = rospy.get_param('~running', False)

    self.init_pub_transforms = rospy.get_param('~pub_transforms', False )
    self.init_create_transforms = rospy.get_param('~create_transforms', False  )

    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    rospy.set_param('~current_folder', self.init_current_folder)

    rospy.set_param('~delay',  self.init_delay)

    rospy.set_param('~pub_transforms',  self.init_pub_transforms)
    rospy.set_param('~create_transforms',  self.init_create_transforms)
    rospy.set_param('~running',self.init_running)

    if do_updates:
      self.updateFromParamServer()
      self.publish_status()


  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = FilePubImgStatus()

    status_msg.home_folder = self.HOME_FOLDER
    current_folder = rospy.get_param('~current_folder', self.init_current_folder)
    status_msg.current_folder = current_folder
    if current_folder == self.HOME_FOLDER:
      selected_folder = 'Home'
    else:
      selected_folder = os.path.basename(current_folder)
    status_msg.selected_folder = selected_folder
    status_msg.current_folders = self.current_folders
    status_msg.supported_file_types = self.SUPPORTED_FILE_TYPES
    status_msg.file_count = self.file_count

    status_msg.current_file_list = self.current_file_list
    status_msg.current_topic_list = self.current_topic_list



    status_msg.min_max_delay = [self.MIN_DELAY, self.MAX_DELAY]
    status_msg.set_delay = rospy.get_param('~delay',  self.init_delay)

    status_msg.pub_transforms = rospy.get_param('~pub_transforms',  self.init_pub_transforms)
    status_msg.create_transforms = rospy.get_param('~create_transforms',  self.init_create_transforms)

    status_msg.running = rospy.get_param('~running',self.init_running)

    self.status_pub.publish(status_msg)


  #############################
  ## APP callbacks

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = rospy.get_param('~current_folder', self.init_current_folder)
    #nepi_msg.publishMsgWarn(self,"Current Folder: " + str(current_folder))
    #nepi_msg.publishMsgWarn(self,"Last Folder: " + str(self.last_folder))
    # Update folder info
    if current_folder != self.last_folder:
      update_status = True
      if os.path.exists(current_folder):
        #nepi_msg.publishMsgWarn(self,"Current Folder Exists")
        current_paths = nepi_ros.get_folder_list(current_folder)
        current_folders = []
        for path in current_paths:
          folder = os.path.basename(path)
          if folder[0] != ".":
            current_folders.append(folder)
        self.current_folders = sorted(current_folders)
        #nepi_msg.publishMsgWarn(self,"Folders: " + str(self.current_folders))
        num_files = 0
        for f_type in self.SUPPORTED_FILE_TYPES:
          num_files = num_files + nepi_ros.get_file_count(current_folder,f_type)
        self.file_count =  num_files
      self.last_folder = current_folder
    # Start publishing if needed
    running = rospy.get_param('~running',self.init_running)
    if running and self.pub_pub == None:
      self.startPub()
      update_status = True
    # Publish status if needed
    if update_status == True:
      self.publish_status()

  def selectFolderCb(self,msg):
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    new_folder = msg.data
    new_path = os.path.join(current_folder,new_folder)
    if os.path.exists(new_path):
      self.last_folder = current_folder
      rospy.set_param('~current_folder',new_path)
    self.publish_status()


  def homeFolderCb(self,msg):
    rospy.set_param('~current_folder',self.HOME_FOLDER)
    self.publish_status()

  def backFolderCb(self,msg):
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    if current_folder != self.HOME_FOLDER:
      new_folder = os.path.dirname(current_folder )
      if os.path.exists(new_folder):
        self.last_folder = current_folder
        rospy.set_param('~current_folder',new_folder)
    self.publish_status()




  #############################
  ## Image callbacks


  def setDelayCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    delay = msg.data
    if delay < self.MIN_DELAY:
      delay = self.MIN_DELAY
    if delay > self.MAX_DELAY:
      delay = self.MAX_DELAY
    rospy.set_param('~delay',delay)
    self.publish_status()

  def startPubCb(self,msg):
    self.startPub()

  def startPub(self):
   if self.pub_pub == None:
      self.pub_pub = rospy.Publisher("~images", Image, queue_size=1, latch=True)
      time.sleep(1)
      current_folder = rospy.get_param('~current_folder', self.init_current_folder)
      # Now start publishing images
      self.file_list = []
      self.num_files = 0
      if os.path.exists(current_folder):
        for f_type in self.SUPPORTED_FILE_TYPES:
          [file_list, num_files] = nepi_ros.get_file_list(current_folder,f_type)
          self.file_list.extend(file_list)
          self.num_files += num_files
          #nepi_msg.publishMsgWarn(self,"File Pub List: " + str(self.file_list))
          #nepi_msg.publishMsgWarn(self,"File Pub Count: " + str(self.num_files))
        if self.num_files > 0:
          self.current_ind = 0
          rospy.Timer(rospy.Duration(1), self.publishCb, oneshot = True)
          running = True
          rospy.set_param('~running',True)
        else:
          nepi_msg.publishMsgInfo(self,"No image files found in folder " + current_folder + " not found")
      else:
        nepi_msg.publishMsgInfo(self,"Folder " + current_folder + " not found")
    self.publish_status()

  def stopPubCb(self,msg):
    running = rospy.get_param('~running',self.init_running)
    running = False
    rospy.set_param('~running',False)
    time.sleep(1)
    if self.pub_pub != None:
      self.pub_pub.unregister()
      time.sleep(1)
      self.pub_pub = None
    self.current_file = "None"
    self.publish_status()


  def publishCb(self,timer):
    running = rospy.get_param('~running',self.init_running)
    if running :
      if self.pub_pub != None:
        # Set current index

        # Check ind bounds
        if self.current_ind > (self.num_files-1):
          self.current_ind = 0 # Start over
        elif self.current_ind < 0:
          self.current_ind = self.num_files-1
        file2open = self.file_list[self.current_ind]
        self.current_file = file2open.split('/')[-1]
        #nepi_msg.publishMsgInfo(self,"Opening File: " + file2open)
 
    running = rospy.get_param('~running',self.init_running)
    if running == True:
      delay = rospy.get_param('~delay',  self.init_delay) -1
      if delay < 0:
        delay == 0
      nepi_ros.sleep(delay)
      rospy.Timer(rospy.Duration(1), self.publishCb, oneshot = True)
    else:
      self.current_ind = 0
      if self.pub_pub != None:
        self.pub_pub.unregister()
        time.sleep(1)
        self.pub_pub = None


               
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self," Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubImgApp()







