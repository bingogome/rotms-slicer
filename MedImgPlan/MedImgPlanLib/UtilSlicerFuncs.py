"""
MIT License

Copyright (c) 2022 Yihao Liu

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

import vtk, math, slicer, json

def setTranslation(p, T):
    T.SetElement(0,3,p[0])
    T.SetElement(1,3,p[1])
    T.SetElement(2,3,p[2])

def setRotation(rotm, T):
    T.SetElement(0,0,rotm[0][0])
    T.SetElement(0,1,rotm[0][1])
    T.SetElement(0,2,rotm[0][2])
    T.SetElement(1,0,rotm[1][0])
    T.SetElement(1,1,rotm[1][1])
    T.SetElement(1,2,rotm[1][2])
    T.SetElement(2,0,rotm[2][0])
    T.SetElement(2,1,rotm[2][1])
    T.SetElement(2,2,rotm[2][2])

def setTransform(rotm, p, T):
    setRotation(rotm, T)
    setTranslation(p, T)

def getRotAndPFromMatrix(T):
    p = [0,0,0]
    p[0],p[1],p[2] = T.GetElement(0,3),T.GetElement(1,3),T.GetElement(2,3)
    mat = [[0,0,0],[0,0,0],[0,0,0]]
    mat[0][0],mat[0][1],mat[0][2] = T.GetElement(0,0),T.GetElement(0,1),T.GetElement(0,2)
    mat[1][0],mat[1][1],mat[1][2] = T.GetElement(1,0),T.GetElement(1,1),T.GetElement(1,2)
    mat[2][0],mat[2][1],mat[2][2] = T.GetElement(2,0),T.GetElement(2,1),T.GetElement(2,2)
    return p, mat

def setColorTextByDistance( \
    view, mesh_p, p, colorchangethresh, \
    indicatorPointOnMesh, \
    indicatorPointPtrtip):

    distarr = [ mesh_p[0]-p[0], mesh_p[1]-p[1], mesh_p[2]-p[2] ]

    dist = math.sqrt( \
        distarr[0] * distarr[0] + distarr[1] * distarr[1] + distarr[2] * distarr[2])
        
    indx = (colorchangethresh - dist) / colorchangethresh \
        if colorchangethresh >= dist else 0.0

    view.cornerAnnotation().SetText(vtk.vtkCornerAnnotation.UpperRight, "Estimate of TRE: {:.4f} mm".format(dist))
    view.cornerAnnotation().GetTextProperty().SetColor(1.0-indx,indx,0)
    view.forceRender()

    indicatorPointOnMesh.GetDisplayNode().SetColor(1.0-indx, indx, 0)
    indicatorPointPtrtip.GetDisplayNode().SetColor(1.0-indx, indx, 0)

def drawAPlane(mat, p, configPath, modelName, transformName, parameterNode):

    with open(configPath+"Config.json") as f:
        configData = json.load(f)

        if not parameterNode.GetNodeReference(modelName):
            planeModel = slicer.util.loadModel(configPath+configData["PLANE_INDICATOR_MODEL"])
            planeModel.GetDisplayNode().SetOpacity(0.3)
            parameterNode.SetNodeReferenceID(modelName, planeModel.GetID())

    if not parameterNode.GetNodeReference(transformName):
        transformNode = slicer.vtkMRMLTransformNode()
        slicer.mrmlScene.AddNode(transformNode)
        parameterNode.SetNodeReferenceID(transformName, transformNode.GetID())

    planeModel = parameterNode.GetNodeReference(modelName)
    transformNode = parameterNode.GetNodeReference(transformName)

    transformMatrix = vtk.vtkMatrix4x4()
    setTransform(mat, p, transformMatrix)

    transformNode.SetMatrixTransformToParent(transformMatrix)
    planeModel.SetAndObserveTransformNodeID(transformNode.GetID())

    slicer.app.processEvents()

def initModelAndTransform(parameterNode, strTransformNode, mtxTransform, strModelNode, fModel):

    if not parameterNode.GetNodeReference(strTransformNode):
        transformNode = slicer.vtkMRMLTransformNode()
        slicer.mrmlScene.AddNode(transformNode)
        parameterNode.SetNodeReferenceID(
            strTransformNode, transformNode.GetID())

    if not parameterNode.GetNodeReference(strModelNode):
        inputModel = slicer.util.loadModel(fModel)
        parameterNode.SetNodeReferenceID(
            strModelNode, inputModel.GetID())

    modelTransform = parameterNode.GetNodeReference(strTransformNode)
    modelIndicator = parameterNode.GetNodeReference(strModelNode)

    modelTransform.SetMatrixTransformToParent(mtxTransform)
    modelIndicator.SetAndObserveTransformNodeID(
        modelTransform.GetID())

    return modelIndicator
