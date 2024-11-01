"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import numpy as np
import pyqtgraph as gl
from rotations import Euler2Rotation
from pyqtgraph.opengl import GLMeshItem
import matplotlib.pyplot as plt

class DrawMav:
    def __init__(self, state, window):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.mav_points, self.mav_meshColors = self.get_points()

        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        self.mav_body = GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.mav_body)  # add body to plot

    def update(self, state):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points.T
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_points(self):
        """"
            Points that define the mav, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define MAV body parameters
        fuse_h = 1
        fuse_w = 1
        fuse_l1 = 2
        fuse_l2 = 1
        fuse_l3 = 4
        wing_l = 1
        wing_w = 6
        tail_h = 1
        tail_l = 1
        tail_w = 2

        # points are in NED coordinates
        # define the points on the aircraft following diagram Fig 2.14
        points = np.array([[fuse_l1, 0,0],#1
            [fuse_l2,fuse_w/2,-fuse_h/2],#2
            [fuse_l2,-fuse_w/2,-fuse_h/2],#3
            [fuse_l2,-fuse_w/2,fuse_h/2],#4
            [fuse_l2,fuse_w/2,fuse_h/2],#5
            [-fuse_l3,0,0],#6
            [0,wing_w/2,0],#7
            [-wing_l, wing_w/2,0],#8
            [-wing_l, -wing_w/2,0],#9
            [0,-wing_w/2,0],#10
            [-fuse_l3+tail_l, tail_w/2, 0],#11
            [-fuse_l3,tail_w/2, 0],#12
            [-fuse_l3,-tail_w/2, 0],#13
            [-fuse_l3+tail_l, -tail_w/2, 0], #14
            [-fuse_l3+tail_l, 0, 0], #15
            [-fuse_l3, 0, -tail_h] #16
            ])
        self.num_points = 16
        self.num_tri_faces = 13
        self.max_pos = 2000
        scale = self.max_pos / 20
        points = scale * points
     



        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((self.num_points,3,4), dtype=np.float32)
        meshColors[0] = red       
        meshColors[1] = red
        meshColors[2] = red
        meshColors[3] = red
        meshColors[4] = blue
        meshColors[5] = blue
        meshColors[6] = blue
        meshColors[7] = green
        meshColors[8] = green
        meshColors[9] = green
        meshColors[10] = green
        meshColors[11] = yellow
        meshColors[12] = yellow
        meshColors[13] = blue
        meshColors[14] = yellow
        meshColors[15] = yellow
        return   points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.empty((self.num_tri_faces*2,3))
        mesh[0] = [0, 1, 2] #1-2-3
        mesh[1] = [0, 1, 4] #1-2-5
        mesh[2] = [0, 3, 4] #1-4-5
        mesh[3] = [0, 2, 3] #1-3-4
        mesh[4] = [5, 1, 2] #6-2-3
        mesh[5] = [5, 1, 4] #6-2-5
        mesh[6] = [5, 2, 3] #6-3-4
        mesh[7] = [5, 3, 4] #6-4-5
        mesh[8] = [6, 7, 8] #7-8-9
        mesh[9] = [6, 8, 9] #7-9-10
        mesh[10] = [5, 14, 15] #6-15-16
        mesh[11] = [10, 11, 12] #11-12-13
        mesh[12] = [10, 12, 13] #11-13-14
        for i in range(self.num_tri_faces):
            for j in range(3):
                mesh[i + self.num_tri_faces][j] = mesh[i][2-j]

        return mesh
def show_points(self):
        fig = plt.figure()
        ax1 = plt.axes(projection = '3d')
        self.mav_scatter(ax1)
        plt.subplots_adjust(left=0.25, bottom=0.25)
        return fig, ax1

