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
import numpy as np
# import matplotlib
# matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import yaml
import time

def plot_ES( docs ):

	# Attributes:
	fontsize = 14

	z_plot 	= np.array(docs["z_plot"])
	f_true 	= np.array(docs["f_true"])
	mpost 	= np.array(docs["mpost"])
	stdpost = np.array(docs["stdpost"])
	Xdata 	= np.array(docs["Xdata"])
	Ydata 	= np.array(docs["Ydata"])
	dH_plot = np.array(docs["dH_plot"])
	EdH_max = np.array(docs["EdH_max"])
	x_next 	= np.array(docs["x_next"])
	mu_next = np.array(docs["mu_next"])

	# ind_max = np.argmax(dH_plot)
	# x_next = z_plot[ind_max]
	# EdH_max = dH_plot[ind_max]

	plt.subplot(211)
	plt.cla()
	plt.grid(True)
	plt.xlim(0.0,1.0)
	plt.ylim(-1,1)

	# GP posterior:
	z_plot_single = z_plot[:,0]
	plt.plot(z_plot_single, mpost, 'r', lw=1)
	plt.fill_between(z_plot_single, mpost-2*stdpost, mpost+2*stdpost, alpha=0.2, color='r')

	# Prior function:
	# plt.plot(z_plot,f_true,'k--')

	# Data:
	plt.plot(Xdata,Ydata,'bo')

	# Next evaluations:
	plt.plot(x_next,mu_next,'go')

	plt.title("Gaussian process posterior", fontsize=fontsize)
	plt.ylabel("Cost", fontsize=fontsize)

	# plt.title("Prior (kernel:  %s)" % kernel, fontsize=12)

	# dH_plot = dH_plot[:200]

	plt.subplot(212)
	plt.cla()
	plt.grid(True)
	plt.xlim(0,1.0)
	plt.ylim(0.0,1.0)
	plt.plot(z_plot_single, dH_plot, 'b')
	# plt.plot(x_next, EdH_max, 'bo')
	plt.title("Expected information gain", fontsize=fontsize)
	plt.xlabel("x", fontsize=fontsize)
	plt.ylabel("E[dH]", fontsize=fontsize)

	plt.draw()
	# plt.pause(10)

	# ax = fig.gca()
	# ax.set_xticks(numpy.arange(0, 1, 0.1))
	# ax.set_yticks(numpy.arange(0, 1., 0.1))


	plt.tight_layout()

	while True :
		plt.pause(1)



def main():

	fig = plt.figure(num=1, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
	plt.ion()
	plt.show()

	stream 		= open("config/input_parameters.yaml", "r")
	node_inpt = yaml.load(stream)
	stream 	= open("data_logging/plot_data.yaml", "r")
	docs 		= yaml.load(stream)
	plot_ES(docs)



	# current_numiter = 0
	# numiter = 0
	# first_time = True
	# while current_numiter < MaxEvals :

	# 	stream 	= open("data_logging/plot_data.yaml", "r")
	# 	docs 		= yaml.load(stream)
	# 	numiter = np.array(docs["numiter"])

	# 	# Update:
	# 	if current_numiter != numiter & numiter != MaxEvals:

	# 		plot_ES(docs)

	# 		current_numiter = numiter
	# 	else:

	# 		# print "No new file found..."
	# 		time.sleep(0.5)

# Call Main:
if __name__ == '__main__':
	main()

