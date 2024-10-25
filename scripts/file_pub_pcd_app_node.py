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
import yaml

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img


from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_pc 

from nepi_app_file_pub_pcd.msg import FilePubPcdStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import PointCloud2

from nepi_ros_interfaces.msg import Frame3DTransform, Frame3DTransformUpdate

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################
# Node File
#########################################

class NepiFilePubPcdApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['pcd']

  #Set Initial Values
  MAX_FILES = 20
  MAX_PUBS = 5
  MIN_DELAY = 0.05
  MAX_DELAY = 5.0
  FACTORY_IMG_PUB_DELAY = 1.0

  UPDATER_DELAY_SEC = 1.0

  ZERO_TRANSFORM_DICT = {
    'x_m': 0, 
    'y_m': 0,
    'z_m': 0,
    'roll_deg': 0,
    'pitch_deg': 0,
    'yaw_deg': 0,
    'heading_deg': 0,
  }
  
  paused = False

  last_folder = ""
  current_folders = []
  current_file_list = []
  current_topic_list = []

  available_files_list = []
  sel_all = False

  running = False
  file_count = 0
  pcd_pub = None

  pcds_dict = dict()
  tf_subs_list = []


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_pcd" # Can be overwitten by luanch command
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
    self.status_pub = rospy.Publisher("~status", FilePubPcdStatus, queue_size=1, latch=True)

    # Start updater process
    rospy.Timer(rospy.Duration(self.UPDATER_DELAY_SEC), self.updaterCb)

    # General File Subscribers
    rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    rospy.Subscriber('~select_folder', String, self.selectFolderCb)
    rospy.Subscriber('~home_folder', Empty, self.homeFolderCb)
    rospy.Subscriber('~back_folder', Empty, self.backFolderCb)

    add_all_sub = rospy.Subscriber('~add_all_pcd_files', Empty, self.addAllFilesCb, queue_size = 10)
    remove_all_sub = rospy.Subscriber('~remove_all_pcd_files', Empty, self.removeAllFilesCb, queue_size = 10)
    add_file_sub = rospy.Subscriber('~add_pcd_file', String, self.addFileCb, queue_size = 10)
    remove_file_sub = rospy.Subscriber('~remove_pcd_file', String, self.removeFileCb, queue_size = 10)


    # Image Pub Scubscirbers and publishers
    rospy.Subscriber('~set_delay', Float32, self.setDelayCb) 
    rospy.Subscriber('~set_pub_transforms', Bool, self.setPubTransformsCb) 
    rospy.Subscriber('~set_create_transforms', Bool, self.setCreateTransformsCb) 
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
    rospy.set_param('~sel_files', [])

    rospy.set_param('~delay', self.FACTORY_IMG_PUB_DELAY)

    rospy.set_param('~pub_transforms',  False)
    rospy.set_param('~create_transforms', False)

    rospy.set_param('~running', False)

    self.publish_status()

  def saveConfigCb(self, msg):  # Just update File init values. Saving done by Config IF system
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

    sel_files = rospy.get_param('~sel_files', ['All'])
    if 'All' in sel_files:
      self.sel_all = True
      time.sleep(1)
    self.init_sel_files = rospy.get_param('~sel_files', [])
        

    self.init_delay = rospy.get_param('~delay', self.FACTORY_IMG_PUB_DELAY)
    self.init_running = rospy.get_param('~running', False)

    self.init_pub_transforms = rospy.get_param('~pub_transforms', False )
    self.init_create_transforms = rospy.get_param('~create_transforms', False  )

    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    rospy.set_param('~current_folder', self.init_current_folder)

    rospy.set_param('~sel_files', self.init_sel_files)

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
    status_msg = FilePubPcdStatus()

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

    status_msg.max_files = self.MAX_FILES
    status_msg.available_files_list = self.available_files_list
    status_msg.selected_files_list = rospy.get_param('~sel_files', self.init_sel_files)

    status_msg.current_file_list = self.current_file_list
    status_msg.current_topic_list = self.current_topic_list


    status_msg.max_pubs = self.MAX_PUBS
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

      if self.sel_all == True:
        self.addAllFiles()
      # Now start publishing images
      self.file_list = []
      self.num_files = 0
      if os.path.exists(current_folder):
        for ind, f_type in enumerate(self.SUPPORTED_FILE_TYPES):
          [file_list, num_files] = nepi_ros.get_file_list(current_folder,f_type)
          self.file_list.extend(file_list)
          self.num_files += num_files
          #nepi_msg.publishMsgWarn(self,"File Pub List: " + str(self.file_list))
          #nepi_msg.publishMsgWarn(self,"File Pub Count: " + str(self.num_files))
        if self.num_files > self.MAX_FILES: 
          self.available_files_list = file_list[:self.MAX_FILES] # Take first MAX_PUBS files
        else:
          self.available_files_list = file_list
        update_sel_files = []
        for count, sel_file in enumerate(file_list):
          if sel_file in self.available_files_list and count < self.MAX_PUBS:
            update_sel_files.append(sel_file)
        rospy.set_param('~sel_files', update_sel_files)
        
    # Start publishing if needed
    running = rospy.get_param('~running',self.init_running)
    if running and self.running == False:
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
    self.sel_all = True
    self.publish_status()


  def homeFolderCb(self,msg):
    rospy.set_param('~current_folder',self.HOME_FOLDER)
    self.sel_all = True
    self.publish_status()

  def backFolderCb(self,msg):
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    if current_folder != self.HOME_FOLDER:
      new_folder = os.path.dirname(current_folder )
      if os.path.exists(new_folder):
        self.last_folder = current_folder
        rospy.set_param('~current_folder',new_folder)
        self.sel_all = True
    self.publish_status()






  #############################
  ## Callbacks



  def addAllFilesCb(self,msg):
    self.addAllFiles()

  def addAllFiles(self):
    ##nepi_msg.publishMsgInfo(self,msg)
    sel_files = self.available_files_list
    if len(sel_files) > self.MAX_PUBS:
      sel_files = sel_files[:self.MAX_PUBS]
    rospy.set_param('~sel_files', sel_files)
    self.publish_status()

  def removeAllFilesCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    nepi_ros.set_param(self,'~sel_files', [])
    self.publish_status()

  def addFileCb(self,msg):
    sel_files = rospy.get_param('~sel_files', self.init_sel_files)
    ##nepi_msg.publishMsgInfo(self,msg)
    file_name = msg.data
    if len(sel_files) < self.MAX_PUBS:
      if file_name in self.available_files_list:
        sel_files.append(file_name)
    rospy.set_param('~sel_files', sel_files)
    self.publish_status()

  def removeFileCb(self,msg):
    sel_files = rospy.get_param('~sel_files', self.init_sel_files)
    ##nepi_msg.publishMsgInfo(self,msg)
    file_name = msg.data
    if file_name in sel_files:
      sel_files.remove(file_name)
    rospy.set_param('~sel_files', sel_files)
    self.publish_status()


  def pausePubCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    self.paused = msg.data
    self.publish_status()


  def setDelayCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    delay = msg.data
    if delay < self.MIN_DELAY:
      delay = self.MIN_DELAY
    if delay > self.MAX_DELAY:
      delay = self.MAX_DELAY
    rospy.set_param('~delay',delay)
    self.publish_status()

  def setPubTransformsCb(self,msg):
    val = msg.data
    rospy.set_param('~pub_transforms',  val)
    self.publish_status()

  def setCreateTransformsCb(self,msg):
    val = msg.data
    rospy.set_param('~create_transforms',  val)
    self.publish_status()

  def startPubCb(self,msg):
    self.startPub()

  def startPub(self):
    create_tfs = rospy.get_param('~create_transforms',  self.init_create_transforms)
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    sel_files = rospy.get_param('~sel_files', self.init_sel_files)
    if self.running == False:
      self.current_file_list = []
      self.current_topic_list = []
      self.pcds_dict = dict()
      for pcd_filename in sel_files:
        pcd_file = os.path.join(current_folder,pcd_filename)
        if os.path.exists(pcd_file):
          pcd_name = os.path.basename(pcd_file)
          pc2_msg = None
          try:
            o3d_pc = o3d.io.read_point_cloud(pcd_file)
            pc2_msg = nepi_pc.o3dpc_to_rospc(o3d_pc)
            pcd_topic_name = os.path.basename(pcd_file).split('.')[0]
            pcd_topic_name = pcd_topic_name.replace('-','_')
            pcd_topic_name = pcd_topic_name.replace('.','')
            pcd_namespace = os.path.join(self.base_namespace,self.node_name,pcd_topic_name)
          except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to read pointcloud from file: " + pcd_file + " " + str(e))
          
          if pc2_msg != None:
            self.pcds_dict[pcd_name] = dict()
            self.pcds_dict[pcd_name]['file'] = pcd_file 
            self.pcds_dict[pcd_name]['topic'] = pcd_topic_name
            self.pcds_dict[pcd_name]['pc2_msg'] = pc2_msg
            nepi_msg.publishMsgInfo(self,"creating publisher for file: " + pcd_file)
            pcd_pub = rospy.Publisher(pcd_namespace, PointCloud2, queue_size=1)
            self.pcds_dict[pcd_name]['pcd_pub'] = pcd_pub
            self.current_file_list.append(pcd_name)
            self.current_topic_list.append(os.path.join(self.node_name,pcd_topic_name))
            # Process Transform Data
            tf_dict = self.ZERO_TRANSFORM_DICT

            transform_file = pcd_file.replace('.pcd','_transform.yaml')
            if os.path.exists(transform_file):
              try:
                with open(transform_file, "r") as file:
                    tf_dict = yaml.safe_load(file)
              except Exception as e:
                tf_dict = self.ZERO_TRANSFORM_DICT
                nepi_msg.publishMsgWarn(self,"Failed to read transform from file: " + transform_file  + " " + str(e))
            else:
              if create_tfs:
                nepi_msg.publishMsgWarn(self,"No transform file found, so creating one")
                try:
                  with open(transform_file, 'w') as f:
                    yaml.dump(tf_dict, f)
                except Exception as e:
                  nepi_msg.publishMsgWarn(self,"Failed to write transform to file: " + transform_file + " " + str(e))
            tf_msg = Frame3DTransform()
            tf_msg.translate_vector.x =  tf_dict['x_m']
            tf_msg.translate_vector.y  =  tf_dict['y_m']
            tf_msg.translate_vector.z  =  tf_dict['z_m']
            tf_msg.rotate_vector.x =  tf_dict['roll_deg']
            tf_msg.rotate_vector.y =  tf_dict['pitch_deg']
            tf_msg.rotate_vector.z =  tf_dict['yaw_deg']
            tf_msg.heading_offset =  tf_dict['heading_deg']  
            self.pcds_dict[pcd_name]['tf_msg'] = tf_msg
            tfu_msg = Frame3DTransformUpdate()
            tfu_msg.topic_namespace = pcd_namespace
            tfu_msg.transform = tfu_msg
            self.pcds_dict[pcd_name]['tfu_msg'] = tfu_msg
            # Find tranform subscribers
            for tf_sub in self.tf_subs_list:
              try:
                tf_sub.unregister()
              except:
                pass
            self.tf_subs_list = []
            tf_subs = nepi_ros.find_topics_by_msg('Frame3DTransformUpdate')
            for tf_sub in tf_subs:
              self.tf_subs_list.append(rospy.Publisher(tf_sub, Frame3DTransformUpdate, queue_size=1))
        else:
          nepi_msg.publishMsgInfo(self,"Could not find file " + pcd_file)
        if len(self.pcds_dict.keys()) > 0:
          nepi_ros.sleep(1,10)
          self.running = True
          rospy.Timer(rospy.Duration(1), self.publishCb, oneshot = True)
          rospy.set_param('~running',True)


    self.publish_status()

  def stopPubCb(self,msg):
    self.stopPub()

  def stopPub(self):
    running = rospy.get_param('~running',self.init_running)
    rospy.set_param('~running',False)
    time.sleep(1)
    for pcd in self.pcds_dict.keys():
      pcd_dict = self.pcds_dict[pcd]
      pcd_pub = pcd_dict['pcd_pub']
      if pcd_pub != None:
        pcd_pub.unregister()
    # unsubscribe tranform subscribers
    for tf_sub in self.tf_subs_list:
      try:
        tf_sub.unregister()
      except:
        pass
    time.sleep(1)
    pcds_dict = dict()
    tf_subs_list = []
    self.current_file_list = []
    self.current_topic_list = []
    self.running = False
    self.publish_status()


  def publishCb(self,timer):
    pub_tfs = rospy.get_param('~pub_transforms',  self.init_pub_transforms)
    running = rospy.get_param('~running',self.init_running)
    pcd_count = len(self.pcds_dict.keys())
    if running and self.paused == False:
      self.running = True
      for pcd_name in self.pcds_dict.keys():
        ros_timestamp = rospy.Time.now()
        pcd_pub = None
        try:
          pcd_pub = self.pcds_dict[pcd_name]['pcd_pub']
          pc2_msg = self.pcds_dict[pcd_name]['pc2_msg']
          pc2_msg.header.stamp = ros_timestamp
          pc2_msg.header.frame_id = 'base_link'

          if not nepi_ros.is_shutdown():
            pcd_pub.publish(pc2_msg)
        except Exception as e:
          nepi_msg.publishMsgWarn(self,"Failed to publish pcd: " + pcd_name + " " + str(e))
        if pub_tfs:
          for tf_sub in self.tf_subs_list:
            try:
              tfu_msg = self.pcds_dict[pcd_name]['tfu_msg']
              tfu_msg.header.stamp = ros_timestamp
              tfu_msg.header.frame_id = 'base_link'
              if not nepi_ros.is_shutdown():
                tf_sub.publish(tfu_msg)
            except Exception as e:
              nepi_msg.publishMsgWarn(self,"Failed to publish pcd: " + pcd_name + " " + str(e))
    running = rospy.get_param('~running',self.init_running)
    if running == True:
      delay = rospy.get_param('~delay',  self.init_delay)
      if delay < 0:
        delay == 0
      nepi_ros.sleep(delay)
      rospy.Timer(rospy.Duration(.001), self.publishCb, oneshot = True)
    else:
      self.stopPub()


               
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self," Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubPcdApp()







