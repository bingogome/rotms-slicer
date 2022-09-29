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
import math
import random

import vtk
import qt
import ctk
import slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin

from MedImgPlanLib.UtilConnections import UtilConnections
from MedImgPlanLib.UtilConnectionsWtNnBlcRcv import UtilConnectionsWtNnBlcRcv
from MedImgPlanLib.UtilFormat import utilNumStrFormat
from MedImgPlanLib.UtilCalculations import mat2quat, rotx, roty, rotz, utilPosePlan
from MedImgPlanLib.UtilSlicerFuncs import drawAPlane, getRotAndPFromMatrix, initModelAndTransform, setColorTextByDistance, setRotation, setTransform, setTranslation

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
        # TODO: add here list of module names that this module requires
        self.parent.dependencies = []
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
        # needed for parameter node observation
        VTKObservationMixin.__init__(self)
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
        self.addObserver(
            slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene,
                         slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
        # (in the selected parameter node).

        self.ui.markupsRegistration.connect(
            "markupsNodeChanged()", self.updateParameterNodeFromGUI)
        self.ui.markupsRegistration.markupsPlaceWidget().setPlaceModePersistency(True)
        self.ui.markupsRegistration.connect(
            'currentMarkupsControlPointSelectionChanged(int)', self.onLandmarkWidgetHilightChange)
        self.ui.markupsToolPosePlan.connect(
            "markupsNodeChanged()", self.updateParameterNodeFromGUI)
        self.ui.markupsToolPosePlan.markupsPlaceWidget().setPlaceModePersistency(True)

        self.ui.comboMeshSelectorSkin.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.updateParameterNodeFromGUI)
        self.ui.comboMeshSelectorBrain.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.updateParameterNodeFromGUI)

        self.ui.checkPlanBrain.connect("toggled(bool)", self.updateParameterNodeFromGUI)

        self.ui.sliderColorThresh.connect(
            "valueChanged(double)", self.updateParameterNodeFromGUI)

        self.ui.sliderManualToolPos.connect(
            "valueChanged(double)", self.updateParameterNodeFromGUI)
        self.ui.sliderManualToolRot.connect(
            "valueChanged(double)", self.updateParameterNodeFromGUI)

        self.ui.sliderGridDistanceApart.connect(
            "valueChanged(double)", self.updateParameterNodeFromGUI)
        self.ui.sliderGridPlanNum.connect(
            "valueChanged(double)", self.updateParameterNodeFromGUI)

        self.ui.checkBoxGridAnatomySurf.connect("toggled(bool)", self.updateParameterNodeFromGUI)
        self.ui.checkBoxGridPerspPlane.connect("toggled(bool)", self.updateParameterNodeFromGUI)

        # Buttons
        self.ui.pushModuleTargetViz.connect('clicked(bool)', self.onPushModuleTargetViz)
        self.ui.pushModuleRobCtrl.connect('clicked(bool)', self.onPushModuleRobCtrl)
        self.ui.pushModuleFreeSurfer.connect('clicked(bool)', self.onPushModuleFreeSurfer)

        self.ui.pushStartTRE.connect('clicked(bool)', self.onPushStartTRE)
        self.ui.pushStopTRE.connect('clicked(bool)', self.onPushStopTRE)

        self.ui.pushPlanLandmarks.connect('clicked(bool)', self.onPushPlanLandmarks)
        self.ui.pushDigHighlighted.connect('clicked(bool)', self.onPushDigHighlighted)
        self.ui.pushDigitize.connect('clicked(bool)', self.onPushDigitize)
        self.ui.pushDigPrev.connect('clicked(bool)', self.onPushDigPrev)
        self.ui.pushDigPrevAndDigHilight.connect('clicked(bool)', self.onPushDigPrevAndDigHilight)
        self.ui.pushRegister.connect('clicked(bool)', self.onPushRegistration)
        self.ui.pushUsePreviousRegistration.connect('clicked(bool)', self.onPushUsePreviousRegistration)
        self.ui.pushToolPosePlan.connect('clicked(bool)', self.onPushToolPosePlan)
        self.ui.pushToolPosePlanRand.connect('clicked(bool)', self.onPushToolPosePlanRand)

        self.ui.pushBackForward.connect('clicked(bool)', self.onPushBackForward)
        self.ui.pushCloseAway.connect('clicked(bool)', self.onPushCloseAway)
        self.ui.pushLeftRight.connect('clicked(bool)', self.onPushLeftRight)
        self.ui.pushPitch.connect('clicked(bool)', self.onPushPitch)
        self.ui.pushRoll.connect('clicked(bool)', self.onPushRoll)
        self.ui.pushYaw.connect('clicked(bool)', self.onPushYaw)

        self.ui.pushPlanGrid.connect('clicked(bool)', self.onPushPlanGrid)
        self.ui.pushGridSetNext.connect('clicked(bool)', self.onPushGridSetNext)
        self.ui.pushGridClear.connect('clicked(bool)', self.onPushGridClear)

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
        self.removeObserver(
            self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

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
            firstMarkupsNode = slicer.mrmlScene.GetFirstNodeByClass(
                "vtkMRMLMarkupsNode")
            if firstMarkupsNode:
                self._parameterNode.SetNodeReferenceID(
                    "LandmarksMarkups", firstMarkupsNode.GetID())

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
            self.removeObserver(
                self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
        self._parameterNode = inputParameterNode
        if self._parameterNode is not None:
            self.addObserver(
                self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

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
        self.ui.markupsRegistration.setCurrentNode(
            self._parameterNode.GetNodeReference("LandmarksMarkups"))
        self.ui.markupsToolPosePlan.setCurrentNode(
            self._parameterNode.GetNodeReference("ToolPoseMarkups"))
        self.ui.comboMeshSelectorSkin.setCurrentNode(
            self._parameterNode.GetNodeReference("InputMeshSkin"))
        self.ui.comboMeshSelectorBrain.setCurrentNode(
            self._parameterNode.GetNodeReference("InputMeshBrain"))
        self.ui.sliderColorThresh.value = float(
            self._parameterNode.GetParameter("ColorChangeThresh"))
        self.ui.sliderGridDistanceApart.value = float(
            self._parameterNode.GetParameter("GridDistanceApart"))
        self.ui.sliderGridPlanNum.value = float(
            self._parameterNode.GetParameter("GridPlanNum"))
        self.ui.checkPlanBrain.checked = (
            self._parameterNode.GetParameter("PlanOnBrain") == "true")
        # self.ui.checkBoxGridAnatomySurf.checked = (
        #     self._parameterNode.GetParameter("PlanGridOnAnatomySurf") == "true")
        self.ui.checkBoxGridPerspPlane.checked = (
            self._parameterNode.GetParameter("PlanGridOnPerspPlane") == "true")

        # Update buttons states and tooltips
        if self._parameterNode.GetNodeReference("LandmarksMarkups"):
            self.ui.pushPlanLandmarks.toolTip = "Feed in all the landmarks"
            self.ui.pushPlanLandmarks.enabled = True
            self.ui.pushDigitize.toolTip = "Start digitizing"
            self.ui.pushDigitize.enabled = True
            self.ui.pushDigHighlighted.toolTip = "Digitize a highlighted landmark"
            self.ui.pushDigHighlighted.enabled = True
            self.ui.pushDigPrevAndDigHilight.toolTip = "Use previous digitization, and \n digitize a highlighted landmark"
            self.ui.pushDigPrevAndDigHilight.enabled = True
            self.ui.pushRegister.toolTip = "Register"
            self.ui.pushRegister.enabled = True
        else:
            self.ui.pushPlanLandmarks.toolTip = "Select landmark markups node first"
            self.ui.pushPlanLandmarks.enabled = False
            self.ui.pushDigitize.toolTip = "Select landmark markups node first"
            self.ui.pushDigitize.enabled = False
            self.ui.pushDigHighlighted.toolTip = "Select landmark markups node first"
            self.ui.pushDigHighlighted.enabled = False
            self.ui.pushDigPrevAndDigHilight.toolTip = "Select landmark markups node first"
            self.ui.pushDigPrevAndDigHilight.enabled = False
            self.ui.pushRegister.toolTip = "Select landmark markups node first"
            self.ui.pushRegister.enabled = False

        if self._parameterNode.GetNodeReference("ToolPoseMarkups"):
            self.ui.pushToolPosePlan.toolTip = "Feed in tool pose"
            self.ui.pushToolPosePlan.enabled = True
            self.ui.pushToolPosePlanRand.toolTip = "Feed in tool pose"
            self.ui.pushToolPosePlanRand.enabled = True
        else:
            self.ui.pushToolPosePlan.toolTip = "Select landmark markups node"
            self.ui.pushToolPosePlan.enabled = False
            self.ui.pushToolPosePlanRand.toolTip = "Select landmark markups node"
            self.ui.pushToolPosePlanRand.enabled = False

        # All the GUI updates are done
        self._updatingGUIFromParameterNode = False

    def updateParameterNodeFromGUI(self, caller=None, event=None):
        """
        This method is called when the user makes any change in the GUI.
        The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
        """

        if self._parameterNode is None or self._updatingGUIFromParameterNode:
            return

        # Modify all properties in a single batch
        wasModified = self._parameterNode.StartModify()

        self._parameterNode.SetNodeReferenceID(
            "InputMeshSkin", self.ui.comboMeshSelectorSkin.currentNodeID)
        self._parameterNode.SetNodeReferenceID(
            "InputMeshBrain", self.ui.comboMeshSelectorBrain.currentNodeID)
        self._parameterNode.SetParameter(
            "ColorChangeThresh", str(self.ui.sliderColorThresh.value))
        self._parameterNode.SetParameter(
            "ManualAdjustToolPoseRot", str(self.ui.sliderManualToolRot.value))
        self._parameterNode.SetParameter(
            "ManualAdjustToolPosePos", str(self.ui.sliderManualToolPos.value))
        self._parameterNode.SetParameter(
            "GridDistanceApart", str(self.ui.sliderGridDistanceApart.value))
        self._parameterNode.SetParameter(
            "GridPlanNum", str(self.ui.sliderGridPlanNum.value))
        self._parameterNode.SetParameter(
            "PlanOnBrain", "true" if self.ui.checkPlanBrain.checked else "false")

        # Grid plan pair
        # self._parameterNode.SetParameter(
        #     "PlanGridOnAnatomySurf", "true" if self.ui.checkBoxGridAnatomySurf.checked else "false")
        self._parameterNode.SetParameter(
            "PlanGridOnPerspPlane", "true" if self.ui.checkBoxGridPerspPlane.checked else "false")

        if self.ui.markupsRegistration.currentNode():
            self._parameterNode.SetNodeReferenceID(
                "LandmarksMarkups", self.ui.markupsRegistration.currentNode().GetID())
        else:
            self._parameterNode.SetNodeReferenceID("LandmarksMarkups", None)
        if self.ui.markupsToolPosePlan.currentNode():
            self._parameterNode.SetNodeReferenceID(
                "ToolPoseMarkups", self.ui.markupsToolPosePlan.currentNode().GetID())
        else:
            self._parameterNode.SetNodeReferenceID("ToolPoseMarkups", None)

        self._parameterNode.EndModify(wasModified)

    def onPushModuleRobCtrl(self):
        slicer.util.selectModule("RobotControl")

    def onPushModuleTargetViz(self):
        slicer.util.selectModule("TargetVisualization")

    def onPushModuleFreeSurfer(self):
        slicer.util.selectModule("FreeSurferImporter")

    def onPushStartTRE(self):
        msg = self.logic._commandsData["START_TRE_CALCULATION_START"]
        try:
            self.logic._connections.utilSendCommand(msg)
        except:
            return
        self.logic.processStartTRECalculation()
        self._parameterNode.SetParameter("TRECalculating", "true")

    def onPushStopTRE(self):
        msg = self.logic._commandsData["START_TRE_CALCULATION_STOP"]
        try:
            self.logic._connections.utilSendCommand(msg)
        except:
            return
        self.logic.processStopTRECalculation()
        self._parameterNode.SetParameter("TRECalculating", "false")

    def onPushPlanLandmarks(self):
        self.updateParameterNodeFromGUI()
        self.logic.processPushPlanLandmarks(
            self._parameterNode.GetNodeReference("LandmarksMarkups"))

    def onPushDigitize(self):
        self.updateParameterNodeFromGUI()
        msg = self.logic._commandsData["START_AUTO_DIGITIZE"]
        self.logic._connections.utilSendCommand(msg)

    def onPushDigHighlighted(self):
        self.updateParameterNodeFromGUI()
        self.logic.processDigHilight()

    def onPushDigPrevAndDigHilight(self):
        self.logic.processDigPrevAndDigHilight()

    def onPushDigPrev(self):
        msg = self.logic._commandsData["START_LANDMARK_DIG_PREV"]
        self.logic._connections.utilSendCommand(msg)

    def onLandmarkWidgetHilightChange(self, idx):
        self._parameterNode.SetParameter("LandmarkWidgetHilightIdx", str(idx))
        print("Highlighted the " + str(idx+1) + "th landmark")

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
        self.logic.processPushToolPosePlan(
            self.ui.markupsToolPosePlan.currentNode())

    def onPushToolPosePlanRand(self):
        self.updateParameterNodeFromGUI()
        if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
            self.logic.processPushToolPosePlan(
                self.ui.markupsToolPosePlan.currentNode())
                
        self.logic.processPushToolPosePlanRand()

    def onPushPlanGrid(self):
        self.updateParameterNodeFromGUI()
        self.logic.processPlanGrid()

    def onPushGridSetNext(self):
        self.updateParameterNodeFromGUI()
        self.logic.processGridSetNext()

    def onPushGridClear(self):
        if self._parameterNode.GetParameter("GridPlanIndicatorNumPrev"):
            prevnum = int(float(self._parameterNode.GetParameter("GridPlanIndicatorNumPrev")))
            for i in range(prevnum):
                slicer.mrmlScene.RemoveNode(self._parameterNode.GetNodeReference("GridPlanTransformNum"+str(i)))
                slicer.mrmlScene.RemoveNode(self._parameterNode.GetNodeReference("GridPlanIndicatorNum"+str(i)))

    def onPushBackForward(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPosePos"))
        self.logic.processManualAdjust([0.0,change,0.0,0.0,0.0,0.0])

    def onPushCloseAway(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPosePos"))
        self.logic.processManualAdjust([0.0,0.0,change,0.0,0.0,0.0])

    def onPushLeftRight(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPosePos"))
        self.logic.processManualAdjust([change,0.0,0.0,0.0,0.0,0.0])

    def onPushPitch(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPoseRot"))
        self.logic.processManualAdjust([0.0,0.0,0.0,change/180.0*math.pi,0.0,0.0])

    def onPushRoll(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPoseRot"))
        self.logic.processManualAdjust([0.0,0.0,0.0,0.0,change/180.0*math.pi,0.0])

    def onPushYaw(self):
        change = float(self._parameterNode.GetParameter("ManualAdjustToolPoseRot"))
        self.logic.processManualAdjust([0.0,0.0,0.0,0.0,0.0,change/180.0*math.pi])

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

    def __init__(self, configPath):
        """
        Called when the logic class is instantiated. Can be used for initializing member variables.
        """
        ScriptedLoadableModuleLogic.__init__(self)
        self._configPath = configPath
        self._connections = MedImgConnections(configPath, "MEDIMG")
        self._connections.setup()
        self._parameterNode = self.getParameterNode()

        with open(self._configPath+"CommandsConfig.json") as f:
            self._commandsData = (json.load(f))["MegImgCmd"]

    def setDefaultParameters(self, parameterNode):
        """
        Initialize parameter node with default settings.
        """
        if not parameterNode.GetParameter("ColorChangeThresh"):
            parameterNode.SetParameter("ColorChangeThresh", "4.0")
        if not parameterNode.GetParameter("ManualAdjustToolPoseRot"):
            parameterNode.SetParameter("ManualAdjustToolPoseRot", "0.0")
        if not parameterNode.GetParameter("ManualAdjustToolPosePos"):
            parameterNode.SetParameter("ManualAdjustToolPosePos", "0.0")
        if not parameterNode.GetParameter("GridDistanceApart"):
            parameterNode.SetParameter("GridDistanceApart", "1.0")
        if not parameterNode.GetParameter("GridPlanNum"):
            parameterNode.SetParameter("GridPlanNum", "16")
        if not parameterNode.GetParameter("TRECalculating"):
            parameterNode.SetParameter("TRECalculating", "false")
        if not parameterNode.GetParameter("PlanOnBrain"):
            parameterNode.SetParameter("PlanOnBrain", "true")
        # if not parameterNode.GetParameter("PlanGridOnAnatomySurf"):
        #     parameterNode.SetParameter("PlanGridOnAnatomySurf", "true")
        if not parameterNode.GetParameter("PlanGridOnPerspPlane"):
            parameterNode.SetParameter("PlanGridOnPerspPlane", "true")

    def processStartTRECalculation(self):
        """
        Called when click the start button
        """
        inModel = self._parameterNode.GetNodeReference("InputMeshSkin")
        if not inModel:
            slicer.util.errorDisplay("Please select a image model first!")
            return

        self._parameterNode = self.getParameterNode()
        self._connections._parameterNode = self.getParameterNode()

        with open(self._configPath+"Config.json") as f:
            configData = json.load(f)

        pointOnMeshIndicator = initModelAndTransform(self._parameterNode, \
            "PointOnMeshTr", self._connections._transformMatrixPointOnMesh, \
            "PointOnMeshIndicator", self._configPath+configData["POINT_INDICATOR_MODEL"])

        pointPtrtipIndicator = initModelAndTransform(self._parameterNode, \
            "PointPtrtipTr", self._connections._transformMatrixPointPtrtip, \
            "PointPtrtipIndicator", self._configPath+configData["POINT_INDICATOR_MODEL"])

        self._connections._pointOnMeshIndicator = pointOnMeshIndicator
        self._connections._pointPtrtipIndicator = pointPtrtipIndicator

        self._connections._colorchangethresh = float(
            self._parameterNode.GetParameter("ColorChangeThresh"))
        self._connections._flag_receiving_nnblc = True
        self._connections.receiveTimerCallBack()

    def processStopTRECalculation(self):

        self._connections._flag_receiving_nnblc = False

    def processPushPlanLandmarks(self, inputMarkupsNode):
        """
        Send out the markups for registration 
        """
        if not inputMarkupsNode:
            slicer.util.errorDisplay("Input markup is invalid!")
            raise ValueError("Input markup is invalid!")
        if inputMarkupsNode.GetNumberOfFiducials() < 3:
            slicer.util.errorDisplay("Input landmarks are less than 3!")
            raise ValueError("Input landmarks are less than 3!")
        self.utilSendLandmarks(-1)

    def processDigHilight(self):
        self.processDigIndividual("START_LANDMARK_DIG_NUM")

    def processDigPrevAndDigHilight(self):
        self.processDigIndividual("START_LANDMARK_DIG_PREV_DIG_HILIGHT")

    def processDigIndividual(self, s):
        if not self._parameterNode.GetParameter("LandmarkWidgetHilightIdx"):
            slicer.util.errorDisplay("Please highlight a landmark first!")
            return
        idx = int(self._parameterNode.GetParameter("LandmarkWidgetHilightIdx"))
        msg = self._commandsData[s] + \
            "_" + str(idx).zfill(2)
        try:
            self._connections.utilSendCommand(msg)
        except:
            return
        print(msg)

    def processPushToolPosePlan(self, inputMarkupsNode):
        """
        Push botton callback function. Plan the pose of the contact point.
        """

        if not inputMarkupsNode:
            slicer.util.errorDisplay("Input markup is invalid!")
            raise ValueError("Input markup is invalid!")

        if inputMarkupsNode.GetNumberOfFiducials() != 4 and \
                inputMarkupsNode.GetNumberOfFiducials() != 3 and \
                inputMarkupsNode.GetNumberOfFiducials() != 2:
            slicer.util.errorDisplay(
                "Input number of landmarks are not 2, 3 or 4!")
            raise ValueError("Input number of landmarks are not 2, 3 or 4!")
        
        p, mat = self.processToolPosePlanByNumOfPoints(inputMarkupsNode)
        self.processToolPosePlanVisualization(p, mat)
        self.processToolPosePlanSend(p, mat)

    def processPushToolPosePlanRand(self):

        targetPoseTransform = self._parameterNode.GetNodeReference(
            "TargetPoseTransform").GetMatrixTransformToParent()
        temp = vtk.vtkMatrix4x4()
        temp.DeepCopy(targetPoseTransform)

        tempOffset = vtk.vtkMatrix4x4()
        tempOffset.SetElement(0,3,random.uniform(-5.0,5.0))
        tempOffset.SetElement(1,3,random.uniform(-5.0,5.0))
        tempOffset.SetElement(2,3,random.uniform(-5.0,5.0))

        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        tempOffset = vtk.vtkMatrix4x4()
        setRotation(rotx(random.uniform(-10.0,10.0)), tempOffset)
        setRotation(roty(random.uniform(-10.0,10.0)), tempOffset)
        setRotation(rotz(random.uniform(-10.0,10.0)), tempOffset)

        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        p, mat = getRotAndPFromMatrix(temp)
        self.processToolPosePlanVisualization(p, mat)
        self.processToolPosePlanSend(p, mat)

    def processToolPosePlanByNumOfPoints(self, inputMarkupsNode):

        a, b, c, p = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

        if inputMarkupsNode.GetNumberOfFiducials() == 4:

            inputMarkupsNode.GetNthFiducialPosition(1, a)
            inputMarkupsNode.GetNthFiducialPosition(2, b)
            inputMarkupsNode.GetNthFiducialPosition(3, c)
            inputMarkupsNode.GetNthFiducialPosition(0, p)
            mat = utilPosePlan(a, b, c, p)
            self._override_y = c

        if inputMarkupsNode.GetNumberOfFiducials() == 3:

            inputMarkupsNode.GetNthFiducialPosition(0, a)
            inputMarkupsNode.GetNthFiducialPosition(1, b)
            inputMarkupsNode.GetNthFiducialPosition(2, c)
            p[0] = (a[0]+b[0]+c[0])/3.0
            p[1] = (a[1]+b[1]+c[1])/3.0
            p[2] = (a[2]+b[2]+c[2])/3.0
            mat = utilPosePlan(a, b, c, p)
            self._override_y = c

        if inputMarkupsNode.GetNumberOfFiducials() == 2:

            override_y = [0, 0, 0]
            inputMarkupsNode.GetNthFiducialPosition(0, p)
            inputMarkupsNode.GetNthFiducialPosition(1, override_y)

            if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):
                inModel = self._parameterNode.GetNodeReference(
                    "InputMeshBrain")
                self._parameterNode.SetNodeReferenceID("BrainMeshOffsetTransform",
                    inModel.GetParentTransformNode().GetID())
                transformFilter = vtk.vtkTransformPolyDataFilter()
                transformFilterTransform = vtk.vtkTransform()
                transformFilterTransform.SetMatrix(
                    self._parameterNode.GetNodeReference("BrainMeshOffsetTransform").GetMatrixTransformToParent())
                transformFilter.SetTransform(transformFilterTransform)
                transformFilter.SetInputData(inModel.GetPolyData())
                transformFilter.Update()
                inModel = transformFilter.GetOutput()

            if (self._parameterNode.GetParameter("PlanOnBrain") == "false"):
                inModel = self._parameterNode.GetNodeReference("InputMeshSkin")
                inModel = inModel.GetPolyData()
            if not inModel:
                slicer.util.errorDisplay("Please select a image model first!")
                return

            cellLocator1 = vtk.vtkCellLocator()
            cellLocator1.SetDataSet(inModel)
            cellLocator1.BuildLocator()
            closestPoint = [0.0, 0.0, 0.0]
            cellObj = vtk.vtkGenericCell()
            cellId, subId, dist2 = vtk.mutable(
                0), vtk.mutable(0), vtk.mutable(0.0)
            cellLocator1.FindClosestPoint(
                p, closestPoint, cellObj, cellId, subId, dist2)

            cellObj.GetPoints().GetPoint(0, a)
            cellObj.GetPoints().GetPoint(1, b)
            cellObj.GetPoints().GetPoint(2, c)

            mat = utilPosePlan(a, b, c, p, override_y)

            self._override_y = override_y
        
        return p, mat

    def processToolPosePlanVisualizationInit(self):

        if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
            transformNode = slicer.vtkMRMLTransformNode()
            transformNodeSingleton = slicer.vtkMRMLTransformNode()
            slicer.mrmlScene.AddNode(transformNode)
            slicer.mrmlScene.AddNode(transformNodeSingleton)
            transformNodeSingleton.SetSingletonTag(
                "MedImgPlan.TargetPoseTransform")
            self._parameterNode.SetNodeReferenceID(
                "TargetPoseTransform", transformNode.GetID())
            self._parameterNode.SetNodeReferenceID(
                "TargetPoseTransformSingleton", transformNodeSingleton.GetID())

        if not self._parameterNode.GetNodeReference("TargetPoseIndicator"):
            with open(self._configPath+"Config.json") as f:
                configData = json.load(f)
            inputModel = slicer.util.loadModel(
                self._configPath+configData["POSE_INDICATOR_MODEL"])
            self._parameterNode.SetNodeReferenceID(
                "TargetPoseIndicator", inputModel.GetID())
            inputModel.GetDisplayNode().SetColor(0, 1, 0)

    def processToolPosePlanProjectTransformOnSkin(self, p, mat):

        if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):

            inModel = self._parameterNode.GetNodeReference("InputMeshSkin")

            cellLocator2 = vtk.vtkCellLocator()
            cellLocator2.SetDataSet(inModel.GetPolyData())
            cellLocator2.BuildLocator()

            ray_point = [0.0, 0.0, 0.0]
            ray_length = 10000.0
            ray_point[0], ray_point[1], ray_point[2] = \
                p[0]+ray_length*mat[0][2], p[1]+ray_length * \
                mat[1][2], p[2]+ray_length*mat[2][2]

            cl_pIntSect, cl_pcoords = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
            cl_t, cl_sub_id = vtk.mutable(0), vtk.mutable(0)
            cellLocator2.IntersectWithLine(
                p, ray_point, 1e-6, cl_t, cl_pIntSect, cl_pcoords, cl_sub_id)

            p[0], p[1], p[2] = cl_pIntSect[0], cl_pIntSect[1], cl_pIntSect[2]

        return p

    def processToolPosePlanVisualization(self, p, mat):

        drawAPlane(mat, p, self._configPath, "PlaneOnMeshIndicator",
            "PlaneOnMeshTransform", self._parameterNode)

        self.processToolPosePlanVisualizationInit()
        p = self.processToolPosePlanProjectTransformOnSkin(p, mat)

        transformMatrix = vtk.vtkMatrix4x4()
        transformMatrixSingleton = vtk.vtkMatrix4x4()

        setTransform(mat, p, transformMatrix)
        setTransform(mat, p, transformMatrixSingleton)

        targetPoseTransform = self._parameterNode.GetNodeReference(
            "TargetPoseTransform")
        targetPoseTransformSingleton = self._parameterNode.GetNodeReference(
            "TargetPoseTransformSingleton")
        targetPoseIndicator = self._parameterNode.GetNodeReference(
            "TargetPoseIndicator")

        targetPoseTransform.SetMatrixTransformToParent(transformMatrix)
        targetPoseTransformSingleton.SetMatrixTransformToParent(
            transformMatrixSingleton)
        targetPoseIndicator.SetAndObserveTransformNodeID(
            targetPoseTransform.GetID())

        slicer.app.processEvents()

    def processToolPosePlanSend(self, p, mat):

        p = self.processToolPosePlanProjectTransformOnSkin(p, mat)
        
        quat = mat2quat(mat)

        msg = self._commandsData["TARGET_POSE_ORIENTATION"] + \
            "_" + utilNumStrFormat(quat[0], 15, 17) + \
            "_" + utilNumStrFormat(quat[1], 15, 17) + \
            "_" + utilNumStrFormat(quat[2], 15, 17) + \
            "_" + utilNumStrFormat(quat[3], 15, 17)

        self._connections.utilSendCommand(msg)

        msg = self._commandsData["TARGET_POSE_TRANSLATION"] + \
            "_" + utilNumStrFormat(p[0]/1000, 15, 17) + \
            "_" + utilNumStrFormat(p[1]/1000, 15, 17) + \
            "_" + utilNumStrFormat(p[2]/1000, 15, 17)

        self._connections.utilSendCommand(msg)


    def utilSendLandmarks(self, curIdx):
        """
        Utility function to recurrantly send landmarks (landmarks on medical image)
        """
        inputMarkupsNode = self._parameterNode.GetNodeReference(
            "LandmarksMarkups")
        numOfFid = inputMarkupsNode.GetNumberOfFiducials()

        if curIdx == numOfFid:
            msg = self._commandsData["LANDMARK_LAST_RECEIVED"]
        elif curIdx != -1:
            ras = [0, 0, 0]
            inputMarkupsNode.GetNthFiducialPosition(curIdx, ras)
            # curIdx is -1, send the current landmark
            # Send in SI units (meter/second/...)
            msg = self._commandsData["LANDMARK_CURRENT_ON_IMG"] + \
                "_" + str(curIdx).zfill(2) + \
                "_" + utilNumStrFormat(ras[0]/1000, 15, 17) + \
                "_" + utilNumStrFormat(ras[1]/1000, 15, 17) + \
                "_" + utilNumStrFormat(ras[2]/1000, 15, 17)
        else:
            # curIdx is -1, send the number of landmarks
            msg = self._commandsData["LANDMARK_NUM_OF_ON_IMG"] + \
                "_" + str(numOfFid).zfill(2)

        print(msg)

        self._connections.utilSendCommand(msg)

        if curIdx <= numOfFid-1:
            curIdx = curIdx+1
            self.utilSendLandmarks(curIdx)

    def processManualAdjust(self, arr):
        if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
            slicer.util.errorDisplay("Please plan tool pose first!")
            return

        targetPoseTransform = self._parameterNode.GetNodeReference(
            "TargetPoseTransform").GetMatrixTransformToParent()
        temp = vtk.vtkMatrix4x4()
        temp.DeepCopy(targetPoseTransform)

        tempOffset = vtk.vtkMatrix4x4()
        tempOffset.SetElement(0,3,arr[0])
        tempOffset.SetElement(1,3,arr[1])
        tempOffset.SetElement(2,3,arr[2])

        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        tempOffset = vtk.vtkMatrix4x4()
        if arr[3]:
            setRotation(rotx(arr[3]), tempOffset)
        if arr[4]:
            setRotation(roty(arr[4]), tempOffset)
        if arr[5]:
            setRotation(rotz(arr[5]), tempOffset)

        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        p, mat = getRotAndPFromMatrix(temp)
        self.processToolPosePlanVisualization(p, mat)
        self.processToolPosePlanSend(p, mat)

    def processGenerateGridIncrementDir(self, n):
        # Output the direction arr of a squre spiral pattern
        # n > 1
        arr, finished_level = [], 0
        for i in range(n):
            if (i+1) == 1:
                arr.append(1)
                finished_level = 1
            else:
                if (i+1) - (2*(finished_level+1)-1) ** 2 == 0:
                    finished_level += 1
                    arr.append(1)
                elif (i+1) - (2*finished_level-1) ** 2 == 1:
                    arr.append(2)
                elif (i+1) - (2*finished_level-1) ** 2 == 2 * (finished_level+1) - 2:
                    arr.append(3)
                elif (i+1) - (2*finished_level-1) ** 2 == 4 * (finished_level+1) - 4:
                    arr.append(4)
                elif (i+1) - (2*finished_level-1) ** 2 == 6 * (finished_level+1) - 6:
                    arr.append(1)
                else: 
                    arr.append(arr[-1])
        return arr

    def processGenerateGridCoordinateArr(self, numOfGrid):

        dist = float(self._parameterNode.GetParameter("GridDistanceApart"))
        arr = self.processGenerateGridIncrementDir(numOfGrid)

        targetPoseTransform = self._parameterNode.GetNodeReference(
            "TargetPoseTransform").GetMatrixTransformToParent()

        temp1, temp = vtk.vtkMatrix4x4(), vtk.vtkMatrix4x4()
        temp1.DeepCopy(targetPoseTransform)
        temp.DeepCopy(targetPoseTransform)

        coor = [temp1]
        arr.pop()

        for i in arr:
            temp2 = vtk.vtkMatrix4x4()
            tempOffset = vtk.vtkMatrix4x4()
            if i == 1:
                tempOffset.SetElement(0,3,dist)
                vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
                temp2.DeepCopy(temp)
                coor.append(temp2)
            if i == 2:
                tempOffset.SetElement(1,3,dist)
                vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
                temp2.DeepCopy(temp)
                coor.append(temp2)
            if i == 3:
                tempOffset.SetElement(0,3,-dist)
                vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
                temp2.DeepCopy(temp)
                coor.append(temp2)
            if i == 4:
                tempOffset.SetElement(1,3,-dist)
                vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
                temp2.DeepCopy(temp)
                coor.append(temp2)
        return coor

    def processClearPrevGridPlan(self):
        if self._parameterNode.GetParameter("GridPlanIndicatorNumPrev"):
            prevnum = int(float(self._parameterNode.GetParameter("GridPlanIndicatorNumPrev")))
            curnum = int(float(self._parameterNode.GetParameter("GridPlanNum")))
            for i in range(prevnum):
                if i>=curnum:
                    slicer.mrmlScene.RemoveNode(self._parameterNode.GetNodeReference("GridPlanTransformNum"+str(i)))
                    slicer.mrmlScene.RemoveNode(self._parameterNode.GetNodeReference("GridPlanIndicatorNum"+str(i)))

    def processVisualizeAndLogPlanGrid(self,coor):

        self.processClearPrevGridPlan()
        with open(self._configPath+"Config.json") as f:
            configData = json.load(f)
        idx = -1

        if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):
            inModel = self._parameterNode.GetNodeReference(
                "InputMeshBrain")
            self._parameterNode.SetNodeReferenceID("BrainMeshOffsetTransform",
                inModel.GetParentTransformNode().GetID())
            transformFilter = vtk.vtkTransformPolyDataFilter()
            transformFilterTransform = vtk.vtkTransform()
            transformFilterTransform.SetMatrix(
                self._parameterNode.GetNodeReference("BrainMeshOffsetTransform").GetMatrixTransformToParent())
            transformFilter.SetTransform(transformFilterTransform)
            transformFilter.SetInputData(inModel.GetPolyData())
            transformFilter.Update()
            inModel = transformFilter.GetOutput()
        if (self._parameterNode.GetParameter("PlanOnBrain") == "false"):
            inModel = self._parameterNode.GetNodeReference("InputMeshSkin")
            inModel = inModel.GetPolyData()
        if not inModel:
            slicer.util.errorDisplay("Please select a image model first!")
            return

        for i in coor:
            idx+=1

            p, mat = getRotAndPFromMatrix(i)
            cellLocator2 = vtk.vtkCellLocator()
            cellLocator2.SetDataSet(inModel)
            cellLocator2.BuildLocator()
            
            ray_length = 0.0
            cl_pIntSect = [float("nan"), float("nan"), float("nan")]
            while math.isnan(sum(cl_pIntSect)) and ray_length<1000:
                ray_point1,ray_point2 = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
                ray_length += 5.0

                ray_point1[0], ray_point1[1], ray_point1[2] = \
                    p[0]+ray_length*mat[0][2], p[1]+ray_length*mat[1][2], p[2]+ray_length*mat[2][2]
                ray_point2[0], ray_point2[1], ray_point2[2] = \
                    p[0]-ray_length*mat[0][2], p[1]-ray_length*mat[1][2], p[2]-ray_length*mat[2][2]

                cl_pIntSect, cl_pcoords = [float("nan"), float("nan"), float("nan")], [0.0, 0.0, 0.0]
                cellObj, cellId, cl_t, cl_sub_id = vtk.vtkGenericCell(), vtk.mutable(0), vtk.mutable(0), vtk.mutable(0)
                cellLocator2.IntersectWithLine(
                    ray_point1, ray_point2, 1e-6, cl_t, cl_pIntSect, cl_pcoords, cl_sub_id, cellId, cellObj)
                    
            p[0], p[1], p[2] = cl_pIntSect[0], cl_pIntSect[1], cl_pIntSect[2]

            a, b, c = [0, 0, 0], [0, 0, 0], [0, 0, 0]
            cellObj.GetPoints().GetPoint(0, a)
            cellObj.GetPoints().GetPoint(1, b)
            cellObj.GetPoints().GetPoint(2, c)

            mat = utilPosePlan(a, b, c, p, self._override_y)

            setTranslation(p,i)
            setRotation(mat,i)

            initModelAndTransform(self._parameterNode, \
                "GridPlanTransformNum"+str(idx), i, \
                "GridPlanIndicatorNum"+str(idx), self._configPath+configData["POSE_INDICATOR_NOTAIL_MODEL"])
            self._parameterNode.GetNodeReference("GridPlanIndicatorNum"+str(idx)).GetDisplayNode().SetColor(0, 0, 1)
            if idx==0:
                self._parameterNode.GetNodeReference("GridPlanIndicatorNum"+str(idx)).GetDisplayNode().SetColor(0, 1, 1)
        self._parameterNode.SetParameter("GridPlanIndicatorNumPrev", self._parameterNode.GetParameter("GridPlanNum"))
        self._parameterNode.SetParameter("GridPlanCurrentAt", "0")

    def processPlanGrid(self):

        if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
            slicer.util.errorDisplay("Please plan tool pose first!")
            raise ValueError("Please plan tool pose first!")

        # if (self._parameterNode.GetParameter("PlanGridOnPerspPlane") == "true") \
        #     and (self._parameterNode.GetParameter("PlanGridOnAnatomySurf") == "true"):
        #         slicer.util.errorDisplay("Please only check one method!")
        #         raise ValueError("Please only check one method!")

        numOfGrid = int(float(self._parameterNode.GetParameter("GridPlanNum")))
        if numOfGrid > 1:

            if (self._parameterNode.GetParameter("PlanGridOnPerspPlane") == "true"):
                coor = self.processGenerateGridCoordinateArr(numOfGrid)
                self.processVisualizeAndLogPlanGrid(coor)

            # if (self._parameterNode.GetParameter("PlanGridOnAnatomySurf") == "true"):

    def processGridSetNext(self):

        numOfGrid = int(float(self._parameterNode.GetParameter("GridPlanNum")))
        if numOfGrid > 1:
            cur = int(float(self._parameterNode.GetParameter("GridPlanCurrentAt")))
            if cur == numOfGrid-1:
                cur = 0
            else:
                cur += 1
            self._parameterNode.SetParameter("GridPlanCurrentAt", str(cur))

            p, mat = getRotAndPFromMatrix(self._parameterNode.GetNodeReference(
                "GridPlanTransformNum"+str(cur)).GetMatrixTransformToParent())
            self.processToolPosePlanVisualization(p, mat)
            self.processToolPosePlanSend(p, mat)    

#
# Use UtilConnectionsWtNnBlcRcv and override the data handler
#


class MedImgConnections(UtilConnectionsWtNnBlcRcv):

    def __init__(self, configPath, modulesufx):
        super().__init__(configPath, modulesufx)
        self._transformMatrixPointOnMesh = None
        self._pointOnMeshIndicator = None
        self._transformMatrixPointPtrtip = None
        self._pointPtrtipIndicator = None

    def setup(self):
        super().setup()
        self._view = slicer.app.layoutManager().threeDWidget(0).threeDView()
        if not self._transformMatrixPointPtrtip:
            self._transformMatrixPointPtrtip = vtk.vtkMatrix4x4()
        if not self._transformMatrixPointOnMesh:
            self._transformMatrixPointOnMesh = vtk.vtkMatrix4x4()

    def handleReceivedData(self):
        """
        Override the parent class function
        """
        p = self.utilMsgParse()
        self.utilPointMsgCallback(p)

    def utilMsgParse(self):
        """
        Msg format: "__msg_point_0000.00000_0000.00000_0000.00000"
            x, y, z
            in mm
        """
        data = self._data_buff.decode("UTF-8")
        msg = data[12:]
        num_str = msg.split("_")
        num = []
        for i in num_str:
            num.append(float(i))
        return num

    def utilPointMsgCallback(self, p):
        """
        Called each time when a valid point message is received
        """
        inModel = self._parameterNode.GetNodeReference("InputMeshSkin")
        pointLocator = vtk.vtkPointLocator()
        pointLocator.SetDataSet(inModel.GetPolyData())
        pointLocator.BuildLocator()
        closest_point = pointLocator.FindClosestPoint(p)
        p_closest = inModel.GetPolyData().GetPoint(closest_point)

        setTranslation(p, self._transformMatrixPointPtrtip)
        setTranslation(p_closest, self._transformMatrixPointOnMesh)

        self._parameterNode.GetNodeReference(
            "PointPtrtipTr").SetMatrixTransformToParent(self._transformMatrixPointPtrtip)
        self._parameterNode.GetNodeReference(
            "PointOnMeshTr").SetMatrixTransformToParent(self._transformMatrixPointOnMesh)

        setColorTextByDistance(
            self._view, p_closest, p, self._colorchangethresh,
            self._pointOnMeshIndicator,
            self._pointPtrtipIndicator)

        slicer.app.processEvents()
