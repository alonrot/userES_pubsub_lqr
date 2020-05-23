# Copyright 2020 Max Planck Society. All rights reserved.
# 
# Author: Alonso Marco Valle (amarcovalle/alonrot) amarco(at)tuebingen.mpg.de
# Affiliation: Max Planck Institute for Intelligent Systems, Autonomous Motion
# Department / Intelligent Control Systems
# 
# This file is part of userES_pubsub_lqr.
# 
# userES_pubsub_lqr is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
# 
# userES_pubsub_lqr is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
# 
# You should have received a copy of the GNU General Public License along with
# userES_pubsub_lqr.  If not, see <http://www.gnu.org/licenses/>.
#
#
'''
======================
3D surface (color map)
======================

Demonstrates plotting a 3D surface colored with the coolwarm color map.
The surface is made opaque by using antialiased=False.

Also demonstrates using the LinearLocator and custom formatting for the
z axis tick labels.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import yaml
import time


stream 	= open("plotting/data/tmp.yaml", "r")
docs 		= yaml.load(stream)

# Attributes:
fontsize = 14

z_plot 	= np.array(docs["z_plot"])
f_true 	= np.array(docs["f_true"])
mpost 	= np.array(docs["mpost"])
stdpost = np.array(docs["stdpost"])
Xdata 	= np.array(docs["Xdata"])
Ydata 	= np.array(docs["Ydata"])
# dH_plot = np.array(docs["dH_plot"])
EdH_max = np.array(docs["EdH_max"])
x_next 	= np.array(docs["x_next"])
mu_next = np.array(docs["mu_next"])

# Error checking:


# z_plot_grid_X = np.reshape(z_plot[:,0],(50,50),order='F')
# z_plot_grid_Y = np.reshape(z_plot[:,1],(50,50),order='F')
X = np.linspace(0,1,50, endpoint=True)
Y = np.linspace(0,1,50, endpoint=True)
z_plot_grid_X, z_plot_grid_Y = np.meshgrid(X, Y)


mpost_grid = np.reshape(mpost,(50,50),order='F')
stdpost_grid = np.reshape(stdpost,(50,50),order='F')

fig = plt.figure(num=1, figsize=(12, 9), dpi=80, facecolor='w', edgecolor='k')
ax = fig.gca(projection='3d')
# plt.ion()
# plt.show()

# Plot the surface.
surf = ax.plot_surface(z_plot_grid_X, z_plot_grid_Y, mpost_grid, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

ax.scatter(Xdata[:,0],Xdata[:,1],Ydata,'z',50,[[1.0,1.0,1.0]])

ax.plot_wireframe(z_plot_grid_X,z_plot_grid_Y,mpost_grid+2*stdpost_grid)
ax.plot_wireframe(z_plot_grid_X,z_plot_grid_Y,mpost_grid-2*stdpost_grid)
ax.set_xlabel('Par 1');
ax.set_ylabel('Par 2');
ax.set_zlabel('Cost');

# print mpost_grid[20:23,20:30]
# print z_plot_grid_X
# print z_plot_grid_Y

# # Make data.
# X = np.arange(-5, 5, 0.25)
# Y = np.arange(-5, 5, 0.25)
# X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X**2 + Y**2)
# Z = np.sin(R)
# surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)


# plt.draw()

# Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
# ax.zaxis.set_major_locator(LinearLocator(10))
# ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
