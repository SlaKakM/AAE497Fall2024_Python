import sys
sys.path.append('..')
import numpy as np
import pyvista as pv
import time


class MAV:
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

    # Points defining the MAV
    points = np.array([
        [fuse_l1, 0, 0],  # 1
        [fuse_l2, fuse_w / 2, -fuse_h / 2],  # 2
        [fuse_l2, -fuse_w / 2, -fuse_h / 2],  # 3
        [fuse_l2, -fuse_w / 2, fuse_h / 2],  # 4
        [fuse_l2, fuse_w / 2, fuse_h / 2],  # 5
        [-fuse_l3, 0, 0],  # 6
        [0, wing_w / 2, 0],  # 7
        [-wing_l, wing_w / 2, 0],  # 8
        [-wing_l, -wing_w / 2, 0],  # 9
        [0, -wing_w / 2, 0],  # 10
        [-fuse_l3 + tail_l, tail_w / 2, 0],  # 11
        [-fuse_l3, tail_w / 2, 0],  # 12
        [-fuse_l3, -tail_w / 2, 0],  # 13
        [-fuse_l3 + tail_l, -tail_w / 2, 0],  # 14
        [-fuse_l3 + tail_l, 0, 0],  # 15
        [-fuse_l3, 0, tail_h]  # 16
    ])
    num_points = 16
    num_tri_faces = 13
    max_pos = 40
    scale = max_pos / 2
    points = scale * points

    faces = np.array([
        [3, 2, 1, 0],
        [3, 0, 1, 4],
        [3, 0, 3, 4],
        [3, 0, 2, 3],
        [3, 5, 1, 2],
        [3, 5, 1, 4],
        [3, 5, 2, 3],
        [3, 5, 3, 4],
        [3, 6, 7, 8],
        [3, 6, 8, 9],
        [3, 5, 14, 15],
        [3, 10, 11, 12],
        [3, 10, 12, 13],
        [3, 5, 14, 15]
    ])

    def __init__(self):
        # Initialize PyVista plotter
        self.plotter = pv.Plotter()
        self.mesh_pv = pv.PolyData(self.points, self.faces)
        self.mav_mesh = self.plotter.add_mesh(self.mesh_pv, color="yellow", show_edges=True)
        self.add_reference_frame()
        self.plotter.show_bounds(grid=True, location='outer', ticks='both')

    def add_reference_frame(self, origin=(0, 0, 0), scale=10.0):
        x_arrow = pv.Arrow(start=origin, direction=(1, 0, 0), scale=scale)
        y_arrow = pv.Arrow(start=origin, direction=(0, 1, 0), scale=scale)
        z_arrow = pv.Arrow(start=origin, direction=(0, 0, 1), scale=scale)
        self.plotter.add_mesh(x_arrow, color='red', label='X-axis')
        self.plotter.add_mesh(y_arrow, color='green', label='Y-axis')
        self.plotter.add_mesh(z_arrow, color='blue', label='Z-axis')

    def run_simulation(self, time_steps=50, dt=0.5):
        velocity = np.array([1, 0.5, 0.2])
        angle_velocity = 0.0002

        # Start the interactive plot
        self.plotter.show(interactive_update=True)

        for t in range(time_steps):
            # Translation
            self.points += velocity * dt

            # Rotation (yaw around Z-axis)
            theta = angle_velocity * t
            rotation_matrix = np.array([
                [np.cos(theta), 0, -np.sin(theta)],
                [0, 1, 0],
                [np.sin(theta), 0, np.cos(theta)]
            ])  
            
            self.points = self.points @ rotation_matrix.T

            # Update the MAV geometry
            self.mesh_pv.points = self.points
            self.plotter.update()

            # Pause for visualization
            time.sleep(dt)




# Create MAV and run simulation
mav = MAV()
mav.run_simulation()

