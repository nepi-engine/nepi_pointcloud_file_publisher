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

//import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"

import CameraViewer from "./CameraViewer"
import {createShortValuesFromNamespaces} from "./Utilities"

import NepiIFSaveData from "./Nepi_IF_SaveData"

@inject("ros")
@observer

// MultiImageViewer Application page
class ImageViewerApp extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_image_viewer",
      appNamespace: null,

      selectedImageTopics: ['None','None','None','None'],
      statusListener: null,
      connected: false,
      needs_update: true
    }
    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)
    this.onChangeInputImgSelection = this.onChangeInputImgSelection.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getSelectedImageTopics = this.getSelectedImageTopics.bind(this)
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
      selectedImageTopics: message.entries
  })

    this.setState({
      connected: true
    })


  }

    // Function for configuring and subscribing to Status
    updateStatusListener() {
      const statusNamespace = this.state.appNamespace + '/status'
      if (this.state.statusListener) {
        this.state.statusListener.unsubscribe()
      }
      var statusListener = this.props.ros.setupStatusListener(
            statusNamespace,
            "nepi_ros_interfaces/StringArray",
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




  // Function for creating image topic options.
  createImageTopicsOptions() {
    var items = []
    items.push(<Option>{"None"}</Option>) 
    const { imageTopics } = this.props.ros
    var imageTopicShortnames = createShortValuesFromNamespaces(imageTopics)
    for (var i = 0; i < imageTopics.length; i++) {
      items.push(<Option value={imageTopics[i]}>{imageTopicShortnames[i]}</Option>)
    }
    return items
  }

  onChangeInputImgSelection(event) {
    const {sendImageSelectionMsg} = this.props.ros
    var imageTopics = this.state.selectedImageTopics
    const namespace = this.getAppNamespace() 
    const selNamespace = namespace + "/set_topic"
    const idx = event.nativeEvent.target.selectedIndex
    const text = event.nativeEvent.target[idx].text
    const value = event.target.value
    if (namespace !== null){    
      var selector_idx = 0
      if (event.nativeEvent.target.id === "ImageSelector_1") {
        selector_idx = 1
      }
      else if (event.nativeEvent.target.id === "ImageSelector_2") {
        selector_idx = 2
      }
      else if (event.nativeEvent.target.id === "ImageSelector_3") {
        selector_idx = 3
      }

      imageTopics[selector_idx] = value
      sendImageSelectionMsg(selNamespace,selector_idx,value)
    }
    this.setState({selectedImageTopics: imageTopics})

  }

  getSelectedImageTopics(){
    const imageTopics = this.state.selectedImageTopics
    return imageTopics
  }

  render() {
    const selectedImageTopics = this.getSelectedImageTopics()
    const appNamespace = this.getAppNamespace()
    const imageOptions = this.createImageTopicsOptions()
    const colCount = ((selectedImageTopics[1] !== 'None') || (selectedImageTopics[2] !== 'None') || (selectedImageTopics[3] !== 'None'))? 3 : 2
    const selectionFlexSize = (colCount === 3)? 0.6 : 0.3
    
    
    return (
      <Columns>
        <Column>

      <Columns>
        <Column>
          <CameraViewer
            imageTopic={selectedImageTopics[0]}
            title={selectedImageTopics[0]}
            hideQualitySelector={false}
          />
          {(selectedImageTopics[2] !== 'None')?
            <CameraViewer
            imageTopic={selectedImageTopics[2]}
            title={selectedImageTopics[2]}
            hideQualitySelector={false}
          />          
          : null
          }
        </Column>
        {(colCount === 3)?
        <Column>
          <CameraViewer
            imageTopic={selectedImageTopics[1]}
            title={selectedImageTopics[1]}
            hideQualitySelector={false}
          />
          {(selectedImageTopics[3] !== 'None')?
            <CameraViewer
              imageTopic={selectedImageTopics[3]}
              title={selectedImageTopics[3]}
              hideQualitySelector={false}
            />          
          : null
          }
        </Column>
        : null
        }

        <Column style={{flex: selectionFlexSize}}>
          <Label title={"Img 1"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_0">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 2"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_1">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 3"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_2">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 4"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_3">
              {imageOptions}
            </Select>
          </Label>
          <div align={"left"} textAlign={"left"}  >
            <ButtonMenu>
              <Button onClick={this.onEventTriggered}>{"Event Trigger"}</Button>
            </ButtonMenu>
          </div>
        </Column>
      </Columns>

      <div hidden={appNamespace === null}>
      <NepiIFSaveData
          saveNamespace={appNamespace}
          title={"Nepi_IF_SaveData"}
      />
      </div>

    </Column>
    </Columns>

    )
  }
}

export default ImageViewerApp
