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

import os, json, logging, math, random
import vtk, qt, ctk, slicer

from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin

from MedImgPlanLib.UtilFormat import utilNumStrFormat
from MedImgPlanLib.UtilCalculations import mat2quat, utilPosePlan, rotx, roty, rotz
from MedImgPlanLib.UtilMedImgConnections import MedImgConnections

from MedImgPlanLib.UtilSlicerFuncs import drawAPlane, getRotAndPFromMatrix, initModelAndTransform, setRotation, setTransform, setTranslation

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
        if not parameterNode.GetParameter("PlanGridOnPerspPlane"):
            parameterNode.SetParameter("PlanGridOnPerspPlane", "false")
        if not parameterNode.GetParameter("ToolRotOption"):
            parameterNode.SetParameter("ToolRotOption", "skin")

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
        
        # Get the position and orientation of the planned tool pose on skin or cortex
        p, mat = self.processToolPosePlanByNumOfPoints(inputMarkupsNode)
        drawAPlane(mat, p, self._configPath, "PlaneOnMeshIndicator",
            "PlaneOnMeshTransform", self._parameterNode)
        self.processToolPosePlanMeshCheck(p, mat)

    def processToolPosePlanMeshCheck(self, p, mat):
        # If planned on skin, then skin search; if planned on cortex, then project on skin
        if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):
            self.processToolPoseParameterNodeSet("TargetPoseTransformCortex", p, mat)
            pSkin, matSkin = self.processSearchForSkinProjection(p, mat)
            self.processToolPoseParameterNodeSet("TargetPoseTransformSkin", pSkin, matSkin)
            drawAPlane(matSkin, pSkin, self._configPath, "PlaneOnMeshSkinIndicator",
                "PlaneOnMeshSkinTransform", self._parameterNode)
            # Considering the shape mismatch of the skin and cortex (brain), use
            # different ways of orientation calculation
            # 1. Depend only on cortex: the orientation is the cortex shape
            # 2. Depend only on skin: the orientation is the skin shape, at the point
            #   projected from the previously planned point
            # 3. Depend on a weighted combination
            # Note, if the pose was planned on skin, then option 2 is the only valid option
            if self._parameterNode.GetParameter("ToolRotOption") == "cortex":
                p = pSkin 
            elif self._parameterNode.GetParameter("ToolRotOption") == "combined":
                slicer.util.errorDisplay("Not implemented yet!")
            else: # Default is the skin
                p = pSkin
                mat = matSkin
                
        self.processToolPoseParameterNodeSet("TargetPoseTransform", p, mat)
        self.processToolPosePlanVisualization()
        self.processToolPosePlanSend(p, mat)
    
    def processToolPosePlanMeshReCheck(self):
        if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):
            targetPoseTransform = self._parameterNode.GetNodeReference(
                "TargetPoseTransformCortex").GetMatrixTransformToParent()
            p, mat = getRotAndPFromMatrix(targetPoseTransform)
            targetPoseTransform = self._parameterNode.GetNodeReference(
                "TargetPoseTransformSkin").GetMatrixTransformToParent()
            pSkin, matSkin = getRotAndPFromMatrix(targetPoseTransform)
            if self._parameterNode.GetParameter("ToolRotOption") == "cortex":
                p = pSkin 
            elif self._parameterNode.GetParameter("ToolRotOption") == "combined":
                slicer.util.errorDisplay("Not implemented yet!")
            else: # Default is the skin
                p = pSkin
                mat = matSkin
            self.processToolPoseParameterNodeSet("TargetPoseTransform", p, mat)
            self.processToolPosePlanVisualization()
            self.processToolPosePlanSend(p, mat)


    def processToolPoseParameterNodeSet(self, nodename, p, mat):

        if not self._parameterNode.GetNodeReference(nodename):
            transformNode = slicer.vtkMRMLTransformNode()
            transformNodeSingleton = slicer.vtkMRMLTransformNode()
            slicer.mrmlScene.AddNode(transformNode)
            slicer.mrmlScene.AddNode(transformNodeSingleton)
            transformNodeSingleton.SetSingletonTag(
                "MedImgPlan." + nodename)
            self._parameterNode.SetNodeReferenceID(
                nodename, transformNode.GetID())
            self._parameterNode.SetNodeReferenceID(
                nodename + "Singleton", transformNodeSingleton.GetID())

        transformMatrix = vtk.vtkMatrix4x4()
        transformMatrixSingleton = vtk.vtkMatrix4x4()
        setTransform(mat, p, transformMatrix)
        setTransform(mat, p, transformMatrixSingleton)
        targetPoseTransform = self._parameterNode.GetNodeReference(
            nodename)
        targetPoseTransformSingleton = self._parameterNode.GetNodeReference(
            nodename + "Singleton")
        targetPoseTransform.SetMatrixTransformToParent(transformMatrix)
        targetPoseTransformSingleton.SetMatrixTransformToParent(
            transformMatrixSingleton)

    def processPushToolPosePlanRand(self):
        if not self._parameterNode.GetNodeReference("TargetPoseTransform"):
            slicer.util.errorDisplay("Please plan tool pose first!")
            return
        targetPoseTransform = self._parameterNode.GetNodeReference(
            "TargetPoseTransform").GetMatrixTransformToParent()
        temp = vtk.vtkMatrix4x4()
        temp.DeepCopy(targetPoseTransform)

        tempOffset = vtk.vtkMatrix4x4()
        pos_range = 15.0
        tempOffset.SetElement(0,3,random.uniform(-pos_range,pos_range))
        tempOffset.SetElement(1,3,random.uniform(-pos_range,pos_range))
        tempOffset.SetElement(2,3,random.uniform(-pos_range,pos_range))

        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        tempOffset = vtk.vtkMatrix4x4()
        ang_range = 15.0/180.0*math.pi
        setRotation(rotx(random.uniform(-ang_range,ang_range)), tempOffset)
        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
        setRotation(roty(random.uniform(-ang_range,ang_range)), tempOffset)
        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)
        setRotation(rotz(random.uniform(-ang_range,ang_range)), tempOffset)
        vtk.vtkMatrix4x4.Multiply4x4(temp,tempOffset,temp)

        p, mat = getRotAndPFromMatrix(temp)
        self.processToolPosePlanVisualization()
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

    def processSearchForSkinProjection(self, pcortex, matcortex):
        """
        Find the projection pose (pos & rot) on the skin. The rot will be 
        the tangential plane on theskin.
        """
        inModel = self._parameterNode.GetNodeReference("InputMeshSkin")

        # Construct cell locator 
        cellLocator = vtk.vtkCellLocator()
        cellLocator.SetDataSet(inModel.GetPolyData())
        cellLocator.BuildLocator()

        # Construct a ray from cortex target point, along the perpendicular direction of cortex
        # at the cortex target point.
        ray_point, ray_length = [0.0, 0.0, 0.0], 10000.0
        ray_point[0], ray_point[1], ray_point[2] = \
            pcortex[0]+ray_length*matcortex[0][2], pcortex[1]+ray_length * \
            matcortex[1][2], pcortex[2]+ray_length*matcortex[2][2]

        # Init some needed parameters.
        cl_pIntSect, cl_pcoords = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        cl_t, cl_sub_id = vtk.mutable(0), vtk.mutable(0)
        a, b, c = [0, 0, 0], [0, 0, 0], [0, 0, 0]
        cellObj = vtk.vtkGenericCell()
        cellId, subId, dist2 = vtk.mutable(0), vtk.mutable(0), vtk.mutable(0.0)
        closestPoint = [0.0, 0.0, 0.0]

        # Search for the projected point on the skin.
        cellLocator.IntersectWithLine(
            pcortex, ray_point, 1e-6, cl_t, cl_pIntSect, cl_pcoords, cl_sub_id)
        pSkin = [cl_pIntSect[0], cl_pIntSect[1], cl_pIntSect[2]]

        # Search for the tangential plane on the skin
        cellLocator.FindClosestPoint(pSkin, closestPoint, cellObj, cellId, subId, dist2)
        cellObj.GetPoints().GetPoint(0, a)
        cellObj.GetPoints().GetPoint(1, b)
        cellObj.GetPoints().GetPoint(2, c)
        matSkin = utilPosePlan(a, b, c, pSkin, self._override_y)

        return pSkin, matSkin

    def processToolPosePlanVisualizationInit(self):
        if not self._parameterNode.GetNodeReference("TargetPoseIndicator"):
            with open(self._configPath+"Config.json") as f:
                configData = json.load(f)
            inputModel = slicer.util.loadModel(
                self._configPath+configData["POSE_INDICATOR_MODEL"])
            self._parameterNode.SetNodeReferenceID(
                "TargetPoseIndicator", inputModel.GetID())
            inputModel.GetDisplayNode().SetColor(0, 1, 0)

    def processToolPosePlanVisualization(self):
        self.processToolPosePlanVisualizationInit()
        targetPoseIndicator = self._parameterNode.GetNodeReference(
            "TargetPoseIndicator")
        targetPoseIndicator.SetAndObserveTransformNodeID(
            self._parameterNode.GetNodeReference("TargetPoseTransform").GetID())
        slicer.app.processEvents()

    def processToolPosePlanSend(self, p, mat):        
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
        self.processToolPoseParameterNodeSet("TargetPoseTransform", p, mat)
        self.processToolPosePlanVisualization()
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

        if (self._parameterNode.GetParameter("PlanOnBrain") == "true"):
            targetPoseTransform = self._parameterNode.GetNodeReference(
                "TargetPoseTransformCortex").GetMatrixTransformToParent()
        else:
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
            drawAPlane(mat, p, self._configPath, "PlaneOnMeshIndicator",
                "PlaneOnMeshTransform", self._parameterNode)
            self.processToolPosePlanMeshCheck(p, mat)
            self.processToolPosePlanVisualization()
            self.processToolPosePlanSend(p, mat)    

