import os
import unittest
import logging
import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
import socket


#
# Connection
#

class MRIPlanConnection():
  """
  Connection class for the MRIPlan module
  """

  def __init__(self):

    # port init
    self._sock_ip_receive = "localhost"
    self._sock_ip_send = "localhost"
    self._sock_receive_port = 8059
    self._sock_send_port = 8057

    self._sock_receive = None
    self._sock_send = None
  
  def setup(self):
    self._sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._sock_receive.bind((self._sock_ip_receive, self._sock_receive_port))
    self._sock_receive.settimeout(0.5)
    
  def clear(self):
    if self._sock_receive:
      self._sock_receive.close()
    if self._sock_send:
      self._sock_send.close()

#
# MRIPlan
#

class MRIPlan(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "MRI Plan" 
    self.parent.categories = ["RoTMS"] 
    self.parent.dependencies = []  # TODO: add here list of module names that this module requires
    self.parent.contributors = ["Yihao Liu (Johns Hopkins University)"] 
    # TODO: update with short description of the module and a link to online module documentation
    self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#MRIPlan">module documentation</a>.
"""
    # TODO: replace with organization, grant and thanks
    self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""

    # Additional initialization step after application startup is complete
    slicer.app.connect("startupCompleted()", appStartUpPostAction)

#
# Register sample data sets in Sample Data module
#

def appStartUpPostAction():
  return

#
# MRIPlanWidget
#

class MRIPlanWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
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
    uiWidget = slicer.util.loadUI(self.resourcePath('UI/MRIPlan.ui'))
    self.layout.addWidget(uiWidget)
    self.ui = slicer.util.childWidgetVariables(uiWidget)

    # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
    # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
    # "setMRMLScene(vtkMRMLScene*)" slot.
    uiWidget.setMRMLScene(slicer.mrmlScene)

    # Create logic class. Logic implements all computations that should be possible to run
    # in batch mode, without a graphical user interface.
    self.logic = MRIPlanLogic()

    # Connections

    # These connections ensure that we update parameter node when scene is closed
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.ui.markupsRegistration.connect("currentNodeChanged(vtkMRMLNode*)", self.updateParameterNodeFromGUI)
    self.ui.markupsCoilPosePlan.connect("currentNodeChanged(vtkMRMLNode*)", self.updateParameterNodeFromGUI)

    # Buttons
    self.ui.pushPlanFiducials.connect('clicked(bool)', self.onPushPlanFiducials)
    self.ui.pushDigitize.connect('clicked(bool)', self.onPushDigitize)
    self.ui.pushRegistration.connect('clicked(bool)', self.onPushRegistration)
    self.ui.pushUsePreviousRegistration.connect('clicked(bool)', self.onPushUsePreviousRegistration)
    self.ui.pushCoilPosePlan.connect('clicked(bool)', self.onPushCoilPosePlan)

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
    if not self._parameterNode.GetNodeReference("FiducialsMarkups"):
      firstMarkupsNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsNode")
      if firstMarkupsNode:
        self._parameterNode.SetNodeReferenceID("FiducialsMarkups", firstMarkupsNode.GetID())

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
    self.ui.markupsRegistration.setCurrentNode(self._parameterNode.GetNodeReference("FiducialsMarkups"))
    self.ui.markupsCoilPosePlan.setCurrentNode(self._parameterNode.GetNodeReference("CoilPoseMarkups"))
    
    # Update buttons states and tooltips
    if self._parameterNode.GetNodeReference("FiducialsMarkups"):
      self.ui.pushPlanFiducials.toolTip = "Feed in all the fiducials"
      self.ui.pushPlanFiducials.enabled = True
      self.ui.pushDigitize.toolTip = "Start digitizing"
      self.ui.pushDigitize.enabled = True
    else:
      self.ui.applyButton.toolTip = "Select fiducial markups node first"
      self.ui.pushPlanFiducials.enabled = False
      self.ui.applyButton.toolTip = "Select fiducial markups node first"
      self.ui.pushDigitize.enabled = False

    if self._parameterNode.GetNodeReference("CoilPoseMarkups"):
      self.ui.pushPlanFiducials.toolTip = "Feed in coil pose"
      self.ui.pushPlanFiducials.enabled = True
    else:
      self.ui.pushPlanFiducials.toolTip = "Select fiducial markups node"
      self.ui.pushPlanFiducials.enabled = False

    # All the GUI updates are done
    self._updatingGUIFromParameterNode = False

  def updateParameterNodeFromGUI(self, caller=None, event=None):
    """
    This method is called when the user makes any change in the GUI.
    The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
    """

    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return

    wasModified = self._parameterNode.StartModify()  # Modify all properties in a single batch

    self._parameterNode.SetNodeReferenceID("FiducialsMarkups", self.ui.markupsRegistration.currentNodeID)
    self._parameterNode.SetNodeReferenceID("CoilPoseMarkups", self.ui.markupsCoilPosePlan.currentNodeID)

    self._parameterNode.EndModify(wasModified)

  def onPushPlanFiducials(self):

    try:
      self.logic.processPushPlanFiducials(self.ui.markupsRegistration.currentNode())

    except Exception as e:
      slicer.util.errorDisplay("Failed to send in results: "+str(e))
      import traceback
      traceback.print_exc()

  def onPushDigitize(self):
    try:
      msg = b"start_autodigitzat"
      self.logic._connections._sock_send.sendto(msg, \
        (self.logic._connections._sock_ip_send, self.logic._connections._sock_send_port))
      try:
        self._connections._sock_receive.recvfrom(512)
      except socket.error:
        raise RuntimeError("timedout.")
    except Exception as e:
      slicer.util.errorDisplay("Failed to send in command: "+str(e))
      import traceback
      traceback.print_exc()

  def onPushRegistration(self):
    try:
      msg = b"start_registration"
      self.logic._connections._sock_send.sendto(msg, \
        (self.logic._connections._sock_ip_send, self.logic._connections._sock_send_port))
      try:
        self._connections._sock_receive.recvfrom(512)
      except socket.error:
        raise RuntimeError("timedout.")
    except Exception as e:
      slicer.util.errorDisplay("Failed to send in command: "+str(e))
      import traceback
      traceback.print_exc()

  def onPushUsePreviousRegistration(self):
    try:
      msg = b"start_useprevregis"
      self.logic._connections._sock_send.sendto(msg, \
        (self.logic._connections._sock_ip_send, self.logic._connections._sock_send_port))
      try:
        self._connections._sock_receive.recvfrom(512)
      except socket.error:
        raise RuntimeError("timedout.")
    except Exception as e:
      slicer.util.errorDisplay("Failed to send in command: "+str(e))
      import traceback
      traceback.print_exc()

  def onPushCoilPosePlan(self):
    
    try:
      self.logic.processPushCoilPosePlan(self.ui.markupsCoilPosePlan.currentNode())

    except Exception as e:
      slicer.util.errorDisplay("Failed to send in results: "+str(e))
      import traceback
      traceback.print_exc()


#
# MRIPlanLogic
#

class MRIPlanLogic(ScriptedLoadableModuleLogic):
  """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self):
    """
    Called when the logic class is instantiated. Can be used for initializing member variables.
    """
    ScriptedLoadableModuleLogic.__init__(self)
    self._connections = MRIPlanConnection()
    self._connections.setup()

  def setDefaultParameters(self, parameterNode):
    """
    Initialize parameter node with default settings.
    """
    self._parameterNode = parameterNode

  def processPushPlanFiducials(self, inputMarkupsNode):
    """
    Send out the markups for registration 
    """

    if not inputMarkupsNode:
      raise ValueError("Input markup is invalid")

    numOfFid = inputMarkupsNode.GetNumberOfFiducials()

    if numOfFid < 3:
      raise ValueError("Input landmarks are less than 3")

    self.utlSendFiducials(-1)

  def utlSendFiducials(self, curIdx):
    """
    Utility function to recurrantly send fiducials(landmarks on MRI)
    """
    inputMarkupsNode = self._parameterNode.GetNodeReference("FiducialsMarkups")
    numOfFid = inputMarkupsNode.GetNumberOfFiducials()
    if curIdx != -1:
      ras = [0,0,0]
      inputMarkupsNode.GetNthFiducialPosition(curIdx,ras)
      # curIdx is -1, send the current fiducial
      msg = b"mri_fid_pnt" + \
        b"_" + str(curIdx).zfill(2).encode('UTF-8') + \
        b"_" + "{:.3f}".format(ras[0]).zfill(10).encode('UTF-8') + \
        b"_" + "{:.3f}".format(ras[1]).zfill(10).encode('UTF-8') + \
        b"_" + "{:.3f}".format(ras[2]).zfill(10).encode('UTF-8')
    else:
      # curIdx is -1, send the number of fiducials
      msg = b"mri_fid_num" + \
        b"_" + str(numOfFid).zfill(2).encode('UTF-8')

    self._connections._sock_send.sendto( \
      msg, (self._connections._sock_ip_send, self._connections._sock_send_port) )
    
    try:
      self._connections._sock_receive.recvfrom(512)
    except socket.error:
      raise RuntimeError("Sending landmarks timedout.")
    else:
      if curIdx < numOfFid-1:
        curIdx = curIdx+1
        self.utlSendFiducials(curIdx)

  def processPushCoilPosePlan(self, inputMarkupsNode):

    if not inputMarkupsNode:
      raise ValueError("Input markup is invalid")

    if inputMarkupsNode.GetNumberOfFiducials() != 4:
      raise ValueError("Input fiducials are not 4")

    a = [0,0,0]
    b = [0,0,0]
    c = [0,0,0]
    p = [0,0,0]
    inputMarkupsNode.GetNthFiducialPosition(1,a)
    inputMarkupsNode.GetNthFiducialPosition(2,b)
    inputMarkupsNode.GetNthFiducialPosition(3,c)
    inputMarkupsNode.GetNthFiducialPosition(0,p)

    quat = self.logic.contactPosePlaneProcess(a,b,c,p)

    

