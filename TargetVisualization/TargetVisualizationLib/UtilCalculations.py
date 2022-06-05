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

import math

def mat2quat(R):
    qw = math.sqrt(1.0+R[0][0]+R[1][1]+R[2][2]) / 2.0 * 4.0
    x = (R[2][1] - R[1][2]) / qw
    y = (R[0][2] - R[2][0]) / qw
    z = (R[1][0] - R[0][1]) / qw
    # print(x,y,z,qw)
    return [x,y,z,qw/4]

def quat2mat(q):
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    mat = [ \
      [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw], \
      [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw], \
      [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]
      ]
    return mat

def normvec3(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)

def transp(A):
    A_ = [[0,0,0],[0,0,0],[0,0,0]]
    for i in [0,1,2]:
      for j in [0,1,2]:
        A_[i][j] = A[j][i]
    return A_
  
def crossProduct(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def utilPosePlan(a,b,c,p):
    """
    Utility function that returns the orientation defined by 3 
    points a, b, c, where the point c is the the direction of 
    x-axis, and z-axis is perpendicular to a-b-c plane. 
    p is the origin of the output pose.
    Output is a rotation matrix.
    """
    n = crossProduct([b[0]-a[0],b[1]-a[1],b[2]-a[2]], [b[0]-c[0],b[1]-c[1],b[2]-c[2]])
    nrm = normvec3(n)
    n = [-n[0]/nrm,-n[1]/nrm,-n[2]/nrm]
    x = crossProduct([c[0]-p[0],c[1]-p[1],c[2]-p[2]], n)
    nrm = normvec3(x)
    x = [x[0]/nrm,x[1]/nrm,x[2]/nrm]
    y = crossProduct(n, x)
    nrm = normvec3(y)
    y = [y[0]/nrm,y[1]/nrm,y[2]/nrm]
    return transp([x, y, n])
     