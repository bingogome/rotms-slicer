"""
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os
import json
import logging

import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin

from MedImgPlanLib.UtilConnections import UtilConnections
from MedImgPlanLib.UtilFormat import utilNumStrFormat
from MedImgPlanLib.UtilCalculations import mat2quat, utilPosePlan
from MedImgPlanLib.UtilSlicerFuncs import setTransform

"""
Check CommandsConfig.json to get UDP messages.
Check Config.json to get program settings.
"""

#
# MedImgPlan
#

class MedImgPlan(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "Medical Image Planning" 
    self.parent.categories = ["RoTMS"] 
    self.parent.dependencies = []  # TODO: add here list of module names that this module requires
    self.parent.contributors = ["Yihao Liu (Johns Hopkins University)"] 
    # TODO: update with short description of the module and a link to online module documentation
    self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https:">module documentation</a>.
"""
    # TODO: replace with organization, grant and thanks
    self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""

    # Additional initialization step after application startup is complete
    slicer.app.connect("startupCompleted()", appStartUpPostAction)

def appStartUpPostAction():
  return

#
# MedImgPlanWidget
#

class MedImgPlanWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent=None):
    """
    Called when the user opens the module the first time and the widget is initialized.
    """
    ScriptedLoadableModuleWidget.__init__(self, parent)
    VTKObservationMixin.__init__(self)  # needed for parameter node observation
    self.logic = None
    self._parameterNode = None
    self._updatingGUIFromParameterNode = False

  def setup(self):
    """
    Called when the user opens the module the first time and the widget is initialized.
    """
    ScriptedLoadableModuleWidget.setup(self)

    # Load widget from .ui file (created by Qt Designer).
    # Additional widgets can be instantiated manually and added to self.layout.
    uiWidget = slicer.util.loadUI(self.resourcePath('UI/MedImgPlan.ui'))
    self.layout.addWidget(uiWidget)
    self.ui = slicer.util.childWidgetVariables(uiWidget)

    # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
    # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
    # "setMRMLScene(vtkMRMLScene*)" slot.
    uiWidget.setMRMLScene(slicer.mrmlScene)

    # Create logic class. Logic implements all computations that should be possible to run
    # in batch mode, without a graphical user interface.
    self.logic = MedImgPlanLogic(self.resourcePath('Configs/'))

    # Connections

    # These connections ensure that we update parameter node when scene is closed
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.ui.markupsRegistration.connect("markupsNodeChanged()", self.updateParameterNodeFromGUI) 
    self.ui.markupsRegistration.markupsPlaceWidget().setPlaceModePersistency(True)
    self.ui.markupsToolPosePlan.connect("markupsNodeChanged()", self.updateParameterNodeFromGUI)
    self.ui.markupsToolPosePlan.markupsPlaceWidget().setPlaceModePersistency(True)

    # Buttons
    self.ui.pushModuleTargetViz.connect('clicked(bool)', self.onPushModuleTargetViz)
    self.ui.pushModuleRobCtrl.connect('clicked(bool)', self.onPushModuleRobCtrl)

    self.ui.pushPlanLandmarks.connect('clicked(bool)', self.onPushPlanLandmarks)
    self.ui.pushDigitize.connect('clicked(bool)', self.onPushDigitize)
    self.ui.pushRegister.connect('clicked(bool)', self.onPushRegistration)
    self.ui.pushUsePreviousRegistration.connect('clicked(bool)', self.onPushUsePreviousRegistration)
    self.ui.pushToolPosePlan.connect('clicked(bool)', self.onPushToolPosePlan)

    # Make sure parameter node is initialized (needed for module reload)
    self.initializeParameterNode()

  def cleanup(self):
    """
    Called when the application closes and the module widget is destroyed.
    """
    self.removeObservers()
    self.logic._connections.clear()

  def enter(self):
    """
    Called each time the user opens this module.
    """
    # Make sure parameter node exists and observed
    self.initializeParameterNode()

  def exit(self):
    """
    Called each time the user opens a different module.
    """
    # Do not react to parameter node changes (GUI wlil be updated when the user enters into the module)
    self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

  def onSceneStartClose(self, caller, event):
    """
    Called just before the scene is closed.
    """
    # Parameter node will be reset, do not use it anymore
    self.setParameterNode(None)

  def onSceneEndClose(self, caller, event):
    """
    Called just after the scene is closed.
    """
    # If this module is shown while the scene is closed then recreate a new parameter node immediately
    if self.parent.isEntered:
      self.initializeParameterNode()

  def initializeParameterNode(self):
    """
    Ensure parameter node exists and observed.
    """
    # Parameter node stores all user choices in parameter values, node selections, etc.
    # so that when the scene is saved and reloaded, these settings are restored.

    self.setParameterNode(self.logic.getParameterNode())

    # Select default input nodes if nothing is selected yet to save a few clicks for the user
    if not self._parameterNode.GetNodeReference("LandmarksMarkups"):
      firstMarkupsNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsNode")
      if firstMarkupsNode:
        self._parameterNode.SetNodeReferenceID("LandmarksMarkups", firstMarkupsNode.GetID())

  def setParameterNode(self, inputParameterNode):
    """
    Set and observe parameter node.
    Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
    """

    if inputParameterNode:
      self.logic.setDefaultParameters(inputParameterNode)

    # Unobserve previously selected parameter node and add an observer to the newly selected.
    # Changes of parameter node are observed so that whenever parameters are changed by a script or any other module
    # those are reflected immediately in the GUI.
    if self._parameterNode is not None:
      self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    self._parameterNode = inputParameterNode
    if self._parameterNode is not None:
      self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

    # Initial GUI update
    self.updateGUIFromParameterNode()

  def updateGUIFromParameterNode(self, caller=None, event=None):
    """
    This method is called whenever parameter node is changed.
    The module GUI is updated to show the current state of the parameter node.
    """

    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return

    # Make sure GUI changes do not call updateParameterNodeFromGUI (it could cause infinite loop)
    self._updatingGUIFromParameterNode = True

    # Update node selectors and sliders
    self.ui.markupsRegistration.setCurrentNode(self._parameterNode.GetNodeReference("LandmarksMarkups"))
    self.ui.markupsToolPosePlan.setCurrentNode(self._parameterNode.GetNodeReference("ToolPoseMarkups"))
    
    # Update buttons states and tooltips
    if self._parameterNode.GetNodeReference("LandmarksMarkups"):
      self.ui.pushPlanLandmarks.toolTip = "Feed in all the landmarks"
      self.ui.pushPlanLandmarks.enabled = True
      self.ui.pushDigitize.toolTip = "Start digitizing"
      self.ui.pushDigitize.enabled = True
      self.ui.pushRegister.toolTip = "Register"
      self.ui.pushRegister.enabled = True
    else:
      self.ui.pushPlanLandmarks.toolTip = "Select landmark markups node first"
      self.ui.pushPlanLandmarks.enabled = False
      self.ui.pushDigitize.toolTip = "Select landmark markups node first"
      self.ui.pushDigitize.enabled = False
      self.ui.pushRegister.toolTip = "Select landmark markups node first"
      self.ui.pushRegister.enabled = False

    if self._parameterNode.GetNodeReference("ToolPoseMarkups"):
      self.ui.pushToolPosePlan.toolTip = "Feed in tool pose"
      self.ui.pushToolPosePlan.enabled = True
    else:
      self.ui.pushToolPosePlan.toolTip = "Select landmark markups node"
      self.ui.pushToolPosePlan.enabled = False

    # All the GUI updates are done
    self._updatingGUIFromParameterNode = False

  def updateParameterNodeFromGUI(self, caller=None, event=None):
    """
    This method is called when the user makes any change in the GUI.
    The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
    """
    # print("updated gui")

    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return

    wasModified = self._parameterNode.StartModify()  # Modify all properties in a single batch

    if self.ui.markupsRegistration.currentNode():
      self._parameterNode.SetNodeReferenceID("LandmarksMarkups", self.ui.markupsRegistration.currentNode().GetID())
    else:
      self._parameterNode.SetNodeReferenceID("LandmarksMarkups", None)
    if self.ui.markupsToolPosePlan.currentNode():
      self._parameterNode.SetNodeReferenceID("ToolPoseMarkups", self.ui.markupsToolPosePlan.currentNode().GetID())
    else:
      self._parameterNode.SetNodeReferenceID("ToolPoseMarkups", None)

    self._parameterNode.EndModify(wasModified)

  def onPushModuleRobCtrl(self):
    slicer.util.selectModule("RobotControl")

  def onPushModuleTargetViz(self):
    slicer.util.selectModule("TargetVisualization")

  def onPushPlanLandmarks(self):
    self.updateParameterNodeFromGUI()
    self.logic.processPushPlanLandmarks(self._parameterNode.GetNodeReference("LandmarksMarkups"))

  def onPushDigitize(self):
    self.updateParameterNodeFromGUI()
    msg = self.logic._commandsData["START_AUTO_DIGITIZE"]
    self.logic._connections.utilSendCommand(msg)

  def onPushRegistration(self):
    self.updateParameterNodeFromGUI()
    msg = self.logic._commandsData["START_REGISTRATION"]
    self.logic._connections.utilSendCommand(msg)
    data = self.logic._connections.receiveMsg()
    print("Registration residual: " + str(data) + "mm")
    slicer.util.infoDisplay("Registration residual: " + str(data) + "mm")

  def onPushUsePreviousRegistration(self):
    self.updateParameterNodeFromGUI()
    msg = self.logic._commandsData["START_USE_PREV_REGISTRATION"]
    self.logic._connections.utilSendCommand(msg)
    data = self.logic._connections.receiveMsg()
    print("Registration residual: " + str(data) + "mm")
    slicer.util.infoDisplay("Registration residual: " + str(data) + "mm")

  def onPushToolPosePlan(self):
    self.updateParameterNodeFromGUI()
    self.logic.processPushToolPosePlan(self.ui.markupsToolPosePlan.currentNode())



#
# MedImgPlanLogic
#

class MedImgPlanLogic(ScriptedLoadableModuleLogic):
  """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self,configPath):
    """
    Called when the logic class is instantiated. Can be used for initializing member variables.
    """
    ScriptedLoadableModuleLogic.__init__(self)
    self._configPath = configPath
    self._connections = UtilConnections(configPath,"MEDIMG")
    self._connections.setup()
    self._parameterNode = self.getParameterNode()
    
    with open(self._configPath+"CommandsConfig.json") as f:
      self._commandsData = (json.load(f))["MegImgCmd"]

  def setDefaultParameters(self, parameterNode):
    """
    Initialize parameter node with default settings.
    """
    return

  def processPushPlanLandmarks(self, inputMarkupsNode):
    """
    Send out the markups for registration 
    """
    if not inputMarkupsNode:
      raise ValueError("Input markup is invalid")
    if inputMarkupsNode.GetNumberOfFiducials() < 3:
      raise ValueError("Input landmarks are less than 3")
    self.utilSendLandmarks(-1)
  
  def processPushToolPosePlan(self, inputMarkupsNode):
    """
    Push botton callback function. Plan the pose of the contact point.
    """

    if not inputMarkupsNode:
      raise ValueError("Input markup is invalid")

    if inputMarkupsNode.GetNumberOfFiducials() != 4 and \
      inputMarkupsNode.GetNumberOfFiducials() != 3:
      raise ValueError("Input landmarks are not 3 or 4")

    a = [0,0,0]
    b = [0,0,0]
    c = [0,0,0]
    p = [0,0,0]

    if inputMarkupsNode.GetNumberOfFiducials() == 4:
      inputMarkupsNode.GetNthFiducialPosition(1,a)
      inputMarkupsNode.GetNthFiducialPosition(2,b)
      inputMarkupsNode.GetNthFiducialPosition(3,c)
      inputMarkupsNode.GetNthFiducialPosition(0,p)
    if inputMarkupsNode.GetNumberOfFiducials() == 3:
      inputMarkupsNode.GetNthFiducialPosition(0,a)
      inputMarkupsNode.GetNthFiducialPosition(1,b)
      inputMarkupsNode.GetNthFiducialPosition(2,c)
      p[0] = (a[0]+b[0]+c[0])/3.0
      p[1] = (a[1]+b[1]+c[1])/3.0
      p[2] = (a[2]+b[2]+c[2])/3.0

    mat = utilPosePlan(a,b,c,p)

    if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
      transformNode = slicer.vtkMRMLTransformNode()
      transformNodeSingleton = slicer.vtkMRMLTransformNode()
      slicer.mrmlScene.AddNode(transformNode)
      slicer.mrmlScene.AddNode(transformNodeSingleton)
      transformNodeSingleton.SetSingletonTag("MedImgPlan.TargetPoseTransform")
      self._parameterNode.SetNodeReferenceID("TargetPoseTransform", transformNode.GetID())
      self._parameterNode.SetNodeReferenceID("TargetPoseTransformSingleton", transformNodeSingleton.GetID())

    if not self._parameterNode.GetNodeReference("TargetPoseIndicator"):
      with open(self._configPath+"Config.json") as f:
        configData = json.load(f)
      inputModel = slicer.util.loadModel(self._configPath+configData["POSE_INDICATOR_MODEL"])
      self._parameterNode.SetNodeReferenceID("TargetPoseIndicator", inputModel.GetID())
      inputModel.GetDisplayNode().SetColor(0,1,0)

    transformMatrix = vtk.vtkMatrix4x4()
    transformMatrixSingleton = vtk.vtkMatrix4x4()
    setTransform(mat, p, transformMatrix)
    setTransform(mat, p, transformMatrixSingleton)

    targetPoseTransform = self._parameterNode.GetNodeReference("TargetPoseTransform")
    targetPoseTransformSingleton = self._parameterNode.GetNodeReference("TargetPoseTransformSingleton")
    targetPoseIndicator = self._parameterNode.GetNodeReference("TargetPoseIndicator")

    targetPoseTransform.SetMatrixTransformToParent(transformMatrix)
    targetPoseTransformSingleton.SetMatrixTransformToParent(transformMatrixSingleton)
    targetPoseIndicator.SetAndObserveTransformNodeID(targetPoseTransform.GetID())

    slicer.app.processEvents()

    quat = mat2quat(mat)

    msg = self._commandsData["TARGET_POSE_ORIENTATION"] + \
      "_" + utilNumStrFormat(quat[0],15,17) + \
      "_" + utilNumStrFormat(quat[1],15,17) + \
      "_" + utilNumStrFormat(quat[2],15,17) + \
      "_" + utilNumStrFormat(quat[3],15,17)

    self._connections.utilSendCommand(msg)

    msg = self._commandsData["TARGET_POSE_TRANSLATION"] + \
      "_" + utilNumStrFormat(p[0]/1000,15,17) + \
      "_" + utilNumStrFormat(p[1]/1000,15,17) + \
      "_" + utilNumStrFormat(p[2]/1000,15,17)

    self._connections.utilSendCommand(msg)

  def utilSendLandmarks(self, curIdx):
    """
    Utility function to recurrantly send landmarks (landmarks on medical image)
    """
    inputMarkupsNode = self._parameterNode.GetNodeReference("LandmarksMarkups")
    numOfFid = inputMarkupsNode.GetNumberOfFiducials()
    
    if curIdx == numOfFid:
      msg = self._commandsData["LANDMARK_LAST_RECEIVED"]
    elif curIdx != -1:
      ras = [0,0,0]
      inputMarkupsNode.GetNthFiducialPosition(curIdx,ras)
      # curIdx is -1, send the current landmark
      # Send in SI units (meter/second/...)
      msg = self._commandsData["LANDMARK_CURRENT_ON_IMG"] + \
        "_" + str(curIdx).zfill(2) + \
        "_" + utilNumStrFormat(ras[0]/1000,15,17) + \
        "_" + utilNumStrFormat(ras[1]/1000,15,17) + \
        "_" + utilNumStrFormat(ras[2]/1000,15,17)
    else:
      # curIdx is -1, send the number of landmarks
      msg = self._commandsData["LANDMARK_NUM_OF_ON_IMG"] + \
        "_" + str(numOfFid).zfill(2)
    
    print(msg)
    
    self._connections.utilSendCommand(msg)

    if curIdx <= numOfFid-1:
      curIdx = curIdx+1
      self.utilSendLandmarks(curIdx)






    
    









    

