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
import ctypes
import numpy as np
import numpy.random as npr

# Transform into numpy array:
def ctype_double_ptr_from_np(arr):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

class CallCostFunction():

	def __init__(self):

		# Handle library object:
		self.hl = ctypes.CDLL('libuserES_pubsub_lqr.so')
		# self.hl = ctypes.CDLL('/is/am/amarcovalle/popspace/devel/lib/libuserES_pubsub_lqr.so')

		# Define the function interface of the exposed functions:
		self.hl.cost_function_init.argtypes = []
		self.hl.cost_function_init.restype = ctypes.c_voidp

		self.hl.cost_function_call.argtypes = [ctypes.POINTER(ctypes.c_double),ctypes.c_int]
		self.hl.cost_function_call.restype = ctypes.c_double

		# Call the function as named in c++:
		self.hl.cost_function_init()

	def get_value(self,pars):

		# Call the function as named in c++:
		my_val = self.hl.cost_function_call(ctype_double_ptr_from_np(pars),1)

		return my_val

if __name__ == "__main__":

	ccf = CallCostFunction()

	my_pars = npr.uniform(low=0.0,high=1.0,size=(2))
	print "my_pars:",my_pars

	my_val = ccf.get_value(my_pars)
	print "my_val=",my_val