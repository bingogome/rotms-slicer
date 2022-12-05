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

#
# Use UtilConnectionsWtNnBlcRcv and override the data handler
#

import slicer, vtk
from MedImgPlanLib.UtilSlicerFuncs import setColorTextByDistance, setTranslation
from MedImgPlanLib.UtilConnectionsWtNnBlcRcv import UtilConnectionsWtNnBlcRcv

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
