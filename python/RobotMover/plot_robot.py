import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.patches import Rectangle
import math

"""
Class to plot the shape of the robot onto a pyplot

"""


class PlotRobot:
    def __init__(self, robot_radius, h_fov):
        self.h_fov = h_fov
        self.robot_radius = robot_radius
        self.fig = plt.gcf()
        self.fig.show()
        self.fig.canvas.draw()
        self.trace_x = []
        self.trace_y = []
        self.trace_angle = []

        # Drawable Var
        self.rect = None
        self.border = None
        self.line_to_cam = None
        self.cone = None
        self.arrow = None

        plt.axis([-1, 1, -1, 1])
        plt.grid(color='gray', linestyle='-', linewidth=0.1)

    def update_plot(self, xpos, ypos, angle, axis=(-100, 100, -100, 100)):
        if axis == (-100, 100, -100, 100):
            axis = (-3+xpos, 3+xpos, -3+ypos, 3+ypos)

        self.trace_x.append(xpos)
        self.trace_y.append(ypos)
        self.trace_angle.append(angle)
        plt.axis(axis)

        # Draw each element
        self.draw_robot_rect()
        self.draw_robot_border()
        # self.draw_robot_cam(cam_x, cam_y)
        # self.draw_robot_cone(cam_x, cam_y, cam_theta)
        self.draw_robot_arrow()

        # Trace odometry
        ax = self.trace_x
        ay = self.trace_y
        plt.plot([ax[-2 % len(ax)], ax[-1]], [ay[-2 % len(ay)], ay[-1]], 'r')

        # Update Figure
        self.fig.canvas.draw()

    # Rectangle for Robot
    def draw_robot_rect(self):
        if self.rect is not None:
            self.rect.remove()

        axes = plt.gca()
        self.rect = Rectangle((self.trace_x[-1]-self.robot_radius, self.trace_y[-1]-self.robot_radius), 2*self.robot_radius, 2*self.robot_radius, angle=self.trace_angle[-1], color='gray', fill=True, alpha=0.8, zorder=10)
        axes.add_patch(self.rect)


    def draw_robot_border(self):
        if self.border is not None:
            self.border.remove()

        axes = plt.gca()
        self.border = Rectangle((self.trace_x[-1]-self.robot_radius, self.trace_y[-1]-self.robot_radius), 2*self.robot_radius, 2*self.robot_radius, angle=self.trace_angle[-1], color='black', fill=False, alpha=0.8, zorder=10)
        axes.add_patch(self.border)

    # Line to Cam
    def draw_robot_cam(self, cam_x, cam_y):
        if self.line_to_cam is not None:
            self.line_to_cam.remove()

        self.line_to_cam, = plt.plot([self.trace_x[-1], cam_x],
                                     [self.trace_y[-1], cam_y],
                                     'y', zorder=15)

    # Cone of View
    def draw_robot_cone(self, cam_x, cam_y, cam_theta):
        axes = plt.gca()
        if self.cone is not None:
            self.cone.remove()

        self.cone = Wedge((cam_x,
                           cam_y),
                          3,
                          math.degrees(cam_theta) - self.h_fov/2,
                          math.degrees(cam_theta) + self.h_fov/2,
                          color="y", alpha=0.2, zorder=10)
        axes.add_patch(self.cone)

    # Arrow
    def draw_robot_arrow(self):
        if self.arrow is not None:
            self.arrow.remove()

        cam_angle = self.trace_angle[-1]

        self.arrow = plt.arrow(self.trace_x[-1] - 0.2 * math.cos(cam_angle),
                               self.trace_y[-1] - 0.2 * math.sin(cam_angle),
                               0.001 * math.cos(cam_angle),
                               0.001 * math.sin(cam_angle),
                               head_length=0.4, head_width=0.25, color='red', zorder=20)



if __name__ == '__main__':
    from pose import Localization
    import time

    plotter = PlotRobot(0.25, 0)
    pose = Localization()
    while True:
        plotter.update_plot(pose.positionX, pose.positionY, pose.positionPitch)
        time.sleep(0.5)
