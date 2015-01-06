# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

import rospy

import numpy as np
import scipy.integrate
import scipy as sp
import matplotlib.pyplot as plt

class Lissajous:
    def __init__(self, size_x=None, size_y=None, period=None):
        # get size from parameter server
        if not size_x:
            self.size_x = float(rospy.get_param('/lissajous/size_x', 0.0))
        else:
            self.size_x = size_x

        if not size_y:
            self.size_y = float(rospy.get_param('/lissajous/size_y', 0.0))
        else:
            self.size_y = size_y

        if (self.size_x <= 0):
            rospy.logerr("Invalid lissajous X size: %s.", self.size_x)
            self.size_x = 0
        if (self.size_y <= 0):
            rospy.logerr("Invalid lissajous Y size: %s.", self.size_y)
            self.size_y = 0
        # get period from parameter server
        if not period:
            self.period = float(rospy.get_param('/lissajous/period', 0.0))
        else:
            self.period = period

        if (self.period <= 0):
            rospy.logerr("Invalid lissajous period %s.", self.period)
            self.period = 0

    def eval(self, t):
        x = self.size_x * np.sin(8 * np.pi * t / self.period)
        y = self.size_y * np.sin(6 * np.pi * t / self.period)
        return np.vstack( [x, y] )

    def lissa_dist(self, t, s):
        assert s.shape == (2,1)
        d = s - self(t)
        return np.sqrt(np.sum(d * d))

    def __call__(self, t):
        return self.eval(t)

    def __getitem__(self, t):
        return self.eval(t)

    def ddt(self, t):
        x = self.size_x * (8 * np.pi / self.period) * \
            np.cos(8 * np.pi * t / self.period)
        y = self.size_y * (6 * np.pi / self.period) * \
            np.cos(6 * np.pi * t / self.period)
        return np.vstack( [x, y] )

    def d2dt2(self, t):
        x = self.size_x * -np.square(8 * np.pi / self.period) * \
            np.sin(8 * np.pi * t / self.period)
        y = self.size_y * -np.square(6 * np.pi / self.period) * \
            np.sin(6 * np.pi * t / self.period)
        return np.vstack( [x, y] )

    def angle(self, t):
        ddt = self.ddt(t)
        return np.arctan2(ddt[1], ddt[0])

    def linear_velocity(self, t):
        return np.sqrt(np.sum(np.square(self.ddt(t)), 0))

    def arc_length(self, t1, t2):
        result = sp.integrate.quad(self.linear_velocity, t1, t2)
        return result[0]

    def curvature(self, t):
        ddt = self.ddt(t)
        d2dt2 = self.d2dt2(t)
        return (ddt[0]*d2dt2[1] - ddt[1]*d2dt2[0]) / \
            np.sqrt(np.power(np.sum(np.square(self.ddt(t)), 0), 3))

    def angular_velocity(self, t):
        return self.linear_velocity(t)*self.curvature(t)

    def plot(self):
        t = np.linspace(0, self.period, 10000)
        p = self.eval(t)
        plt.scatter(p[0], p[1], s = 10, c = t, lw = 0)
        plt.colorbar()
        plt.title('Lissajous (color by time)')
        plt.axis('equal')
        plt.show()

    def plot_linear_velocity(self):
        t = np.linspace(0, self.period, 10000)
        p = self.eval(t)
        plt.scatter(p[0], p[1], s = 10, c = self.linear_velocity(t), lw = 0)
        plt.colorbar()
        plt.title('Linear Velocity (meters/second)')
        plt.axis('equal')
        plt.show()

    def plot_angular_velocity(self):
        t = np.linspace(0, self.period, 10000)
        p = self.eval(t)
        plt.scatter(p[0], p[1], s = 10, c = self.angular_velocity(t)*180/np.pi, lw = 0)
        plt.colorbar()
        plt.title('Angular Velocity (degrees/second)')
        plt.axis('equal')
        plt.show()

