import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sin, cos, radians
import numpy as np
import scipy.spatial
from scipy.spatial.transform import Rotation as R

def calcular_centroide( points ):
    centroid = np.array([0,0,0],dtype=float)
    for i in range(0,12):
      centroid[0] += points[0,i]
      centroid[1] += points[1,i]
      centroid[2] += points[2,i]
    
    centroid[0] /= 12
    centroid[1] /= 12
    centroid[2] /= 12

    return np.reshape(centroid,(3,1))

yaw = 0
pitch = 0
roll = 0
translation = np.array([0,0.75,0])

# Transform to radians
yaw = radians(yaw)
pitch = radians(pitch)
roll = radians(roll)

# Construct rotation matrices
Rx = np.array([[1,0,0],[0,cos(roll),-sin(roll)],[0,sin(roll),cos(roll)]])
Ry = np.array([[cos(pitch),0,sin(pitch)],[0,1,0],[-sin(pitch),0,cos(pitch)]])
Rz = np.array([[cos(yaw),-sin(yaw),0],[sin(yaw),cos(yaw),0],[0,0,1]])

# Combine rotation matrices
# R = Rz @ Ry @ Rx
R = np.matmul(Rz,np.matmul(Ry,Rx))

# Create homogenous transformation matrix
T = np.zeros((4,4))
T[0:3,0:3] = R
T[0:3,3] = translation
T[3,3] = 1

point_this = np.array([
    [2.0, 2.0,  2.0, 2.0, 2.0, 2.0, 2.0,  2.0,  2.0,  2.0,  2.0,  2.0],
    [0.5, 0.5,  0.5, 0.5, 0.5, 0.5, 0.65, 0.65, 0.65, 0.35, 0.35, 0.35],
    [0.5, 0.35, 0.2, 0.1, 0.0,-0.2, 0.2, -0.1, -0.3,  0.2, -0.1, -0.3]
    ])

point_this = np.append(point_this,np.ones((1,12)),axis=0)
# print(this_points)

point_other = np.matmul(T,point_this)

# to cartesian
point_this  = point_this[0:3,:]
point_other = point_other[0:3,:]

centroid_this  = calcular_centroide(point_this)
centroid_other = calcular_centroide(point_other)

# print(centroid_this.shape)
# print(centroid_other.shape)

S = np.zeros((3,3),dtype=float)

point_this = point_this-centroid_this
point_other = point_other-centroid_other

for i in range(0,point_this.shape[1]):
    S[0,0] += point_other[0,i] * point_this[0,i]
    S[0,1] += point_other[0,i] * point_this[1,i]
    S[0,2] += point_other[0,i] * point_this[2,i]

    S[1,0] += point_other[1,i] * point_this[0,i]
    S[1,1] += point_other[1,i] * point_this[1,i]
    S[1,2] += point_other[1,i] * point_this[2,i]

    S[2,0] += point_other[2,i] * point_this[0,i]
    S[2,1] += point_other[2,i] * point_this[1,i]
    S[2,2] += point_other[2,i] * point_this[2,i]

N = np.zeros((4,4),dtype=float)

N[0,0] = S[0,0] + S[1,1] + S[2,2]
N[0,1] = S[1,2] - S[2,1]
N[0,2] = S[2,0] - S[0,2]
N[0,3] = S[0,1] - S[1,0]

N[1,0] = N[0,1]
N[1,1] = S[0,0] - S[1,1] - S[2,2]
N[1,2] = S[0,1] + S[1,0]
N[1,3] = S[2,0] + S[0,2]

N[2,0] =  N[0,2]
N[2,1] =  N[1,2]
N[2,2] = -S[0,0] + S[1,1] - S[2,2]
N[2,3] =  S[1,2] + S[2,1]

N[3, 0] =  N[0,3] + S[2,2]
N[3, 1] =  N[1,3]
N[3, 2] =  N[2,3]
N[3, 3] = -S[0,0] - S[1,1] + S[2,2]

w, v = np.linalg.eig(N)
print("w=",w)
print("v=",v)

idx = np.argmin(w)
q = v[:,idx] # quaternion

r = R.from_quat(q)
print(r)

'''
Matrix3f S;	// Zeroed by default
    for (size_t i = 0; i < points_other.size(); i++)
    {
      points_this[i] -= ct_this;
      points_other[i] -= ct_others;

      ROS_INFO("point_this: %.3f, %.3f, %.3f", points_this[i][0], points_this[i][1], points_this[i][2]);
      ROS_INFO("point_other: %.3f, %.3f, %.3f", points_other[i][0], points_other[i][1], points_other[i][2]);

      S(0, 0) += points_other[i][0] * points_this[i][0];
      S(0, 1) += points_other[i][0] * points_this[i][1];
      S(0, 2) += points_other[i][0] * points_this[i][2];

      S(1, 0) += points_other[i][1] * points_this[i][0];
      S(1, 1) += points_other[i][1] * points_this[i][1];
      S(1, 2) += points_other[i][1] * points_this[i][2];

      S(2, 0) += points_other[i][2] * points_this[i][0];
      S(2, 1) += points_other[i][2] * points_this[i][1];
      S(2, 2) += points_other[i][2] * points_this[i][2];
    }
'''
'''
# print(other_points)

# Create figure
fig = plt.figure()

# Prepare figure for 3D data
ax = fig.gca(projection='3d')

# Name axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Plot points
ax.scatter(this_points[0,:], this_points[1,:], this_points[2,:])
ax.scatter(0, 0, 0, color="red")
ax.scatter(other_points[0,:], other_points[1,:], other_points[2,:], color="black")

fig.show()
plt.pause(0.01)
input("<Hit Enter To Close>")
'''


# T = np.array(4,4)