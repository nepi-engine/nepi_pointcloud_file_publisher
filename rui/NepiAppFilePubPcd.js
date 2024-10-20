/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Button, { ButtonMenu } from "./Button"
import Label from "./Label"
import Input from "./Input"
import Toggle from "react-toggle"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"


import CameraViewer from "./CameraViewer"

import {createShortUniqueValues, onDropdownSelectedSendStr, createMenuListFromStrList} from "./Utilities"
import {createShortValuesFromNamespaces} from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

@inject("ros")
@observer

class FilePubPcdApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
		
      appName: 'app_file_pub_pcd',
	    appNamespace: null,

      viewableFolders: false,

      home_folder: 'None',
      current_folder: null,
      selected_folder: 'Home',
      current_folders: [],
      supported_file_types: [],
      selected_file: 'Home',
      file_count: 0,
      max_pubs: 5,
      current_file_list: [],
      current_topic_list: [],

      paused: false,


      min_max_delay: 1,
      set_delay: 1,

      pub_transforms: false,
      create_transforms: false,

      pub_running: false,

      statusListener: null,
      connected: false,
      needs_update: true

    }

    this.createFolderOptions = this.createFolderOptions.bind(this)
    this.onChangeFolderSelection = this.onChangeFolderSelection.bind(this)
    this.toggleViewableFolders = this.toggleViewableFolders.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)


  }


  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      home_folder: message.home_folder ,
      current_folders: message.current_folders ,
      selected_folder: message.selected_folder,
      supported_file_types: message.supported_file_types,
      file_count: message.file_count ,
      max_pubs: message.max_pubs,
      current_file_list: message.current_file_list ,
      current_topic_list: message.current_topic_list ,
      
      paused: message.paused ,

      min_max_delay: message.min_max_delay ,
      set_delay: message.set_delay ,

      pub_transforms: message.pub_transforms,
      create_transforms: message.create_transforms,

      pub_running: message.running
  })

  var current_folder = 'None'
  if (message.current_folder === message.home_folder ){
    current_folder = 'Home'
  }
  else {
    current_folder = message.current_folder
  }


  this.setState({
      current_folder: current_folder,
      connected: true
    })


  }

    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const namespace = this.getAppNamespace()
      const statusNamespace = namespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_app_file_pub_pcd/FilePubPcdStatus",
            this.statusListener
          )
      this.setState({ 
        statusListener: statusListener,
        needs_update: false
      })
    }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (prevState.appNamespace !== namespace && namespace !== null)
    const needs_update = (this.state.needs_update && namespace !== null)
    if (namespace_updated || needs_update) {
      if (namespace.indexOf('null') === -1){
        this.setState({appNamespace: namespace})
        this.updateStatusListener()
      } 
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }



  renderPubControls() {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const pubRunning = this.state.pub_running
    const appImageTopic = pubRunning === true ? this.state.appNamespace + "/images" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    return (


    <Columns>
    <Column>


        <div hidden={!this.state.connected}>

          <Label title={"Publishing"}>
              <BooleanIndicator value={pubRunning} />
            </Label>

              <div hidden={pubRunning}>
            <ButtonMenu>
              <Button 
                disabled={pubRunning}
                onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/start_pub")}>{"Start Publishing"}</Button>
            </ButtonMenu>
            </div>

            <div hidden={!pubRunning}>
            <ButtonMenu>
              <Button onClick={() => this.props.ros.sendTriggerMsg(appNamespace + "/stop_pub")}>{"Stop Publishing"}</Button>
            </ButtonMenu>
            </div>


            <Label title={"Current Folder"}>
            <Input disabled value={this.state.current_folder} />
            </Label>

            <Label title={"Pcd File Count"}>
            <Input disabled value={this.state.file_count} />
            </Label>

        </div>

        </Column>
        </Columns>


    )
  }




  // Function for creating image topic options.
  createFolderOptions() {
    const cur_folder = this.state.current_folder
    const sel_folder = this.state.selected_folder
    var items = []
    if (cur_folder){
      items.push(<Option value={"Home"}>{"Home"}</Option>) 
      if (sel_folder !== 'Home'){
        items.push(<Option value={"Back"}>{"Back"}</Option>) 
      }
      const folders = this.state.current_folders
      for (var i = 0; i < folders.length; i++) {
        items.push(<Option value={folders[i]}>{folders[i]}</Option>)
      }
    }
    return items
  }

  onChangeFolderSelection(event) {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const namespace = this.state.appNamespace
    const setNamespace = namespace + "/select_folder"
    const homeNamespace = namespace + "/home_folder"
    const backNamespace = namespace + "/back_folder"
    const home_folder = this.state.home_folder
    const value = event.target.value
    if (namespace !== null){    
      var selector_idx = 0
      if (value === 'Home') {
        sendTriggerMsg(homeNamespace)
      }
      else if (value === 'Back') {
        sendTriggerMsg(backNamespace)
      }
      else {
        sendStringMsg(setNamespace,value)
      }
    }
    this.setState({selected_folder: value})
  }



  toggleViewableFolders() {
    const viewable = !this.state.viewableFolders
    this.setState({viewableFolders: viewable})
  }


 render() {
    const {sendTriggerMsg, sendStringMsg} = this.props.ros
    const appNamespace = this.state.appNamespace
    const folderOptions = this.createFolderOptions()
    const pubRunning = this.state.pub_running
    const appImageTopic = pubRunning === true ? this.state.appNamespace + "/images" : null
    const viewableFolders = (this.state.viewableFolders || pubRunning === false)
    return (

    <Columns>
      <Column>

          <CameraViewer
            imageTopic={appImageTopic}
            title={this.state.imageText}
            hideQualitySelector={false}
          />


    </Column>
    <Column>


        <Columns>
        <Column>


            <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            {"Select Folder"}
          </label>

            <div onClick={this.toggleViewableFolders} style={{backgroundColor: Styles.vars.colors.grey0}}>
              <Select style={{width: "10px"}}/>
            </div>
            <div hidden={viewableFolders === false}>
            {folderOptions.map((folder) =>
            <div onClick={this.onChangeFolderSelection}>
              <body value = {folder} style={{color: Styles.vars.colors.black}}>{folder}</body>
            </div>
            )}
            </div>
    

        </Column>
        <Column>

        {this.renderPubControls()}

        </Column>
        </Columns>


        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        <div hidden={!this.state.connected}>

        <Columns>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg( appNamespace + "/reset_app")}>{"Reset App"}</Button>
          </ButtonMenu>

        </Column>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/reset_config")}>{"Reset Config"}</Button>
          </ButtonMenu>

        </Column>
        <Column>

          <ButtonMenu>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/save_config")}>{"Save Config"}</Button>
          </ButtonMenu>

        </Column>
        </Columns>
      
       </div>



  </Column>
    </Columns>

    )
  }

}

export default FilePubPcdApp
