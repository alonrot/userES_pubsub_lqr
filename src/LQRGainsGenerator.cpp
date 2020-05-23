// Copyright 2020 Max Planck Society. All rights reserved.
// 
// Author: Alonso Marco Valle (amarcovalle/alonrot) amarco(at)tuebingen.mpg.de
// Affiliation: Max Planck Institute for Intelligent Systems, Autonomous Motion
// Department / Intelligent Control Systems
// 
// This file is part of userES_pubsub_lqr.
// 
// userES_pubsub_lqr is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option) any
// later version.
// 
// userES_pubsub_lqr is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
// 
// You should have received a copy of the GNU General Public License along with
// userES_pubsub_lqr.  If not, see <http://www.gnu.org/licenses/>.
//
//
#include "LQRGainsGenerator.hpp"

LQRGainsGenerator::LQRGainsGenerator(	std::string topic_lqr_gains,
																			std::string topic_lqr_gains_id,
																			int lqr_gains_id,
																			size_t Nx, size_t Nu,
																			std::string label_dim0, std::string label_dim1,
																			bool full_verbosity,
																			Eigen::MatrixXd A, Eigen::MatrixXd B, 
																			Eigen::MatrixXd N, 
																			Eigen::VectorXd Q_design, Eigen::VectorXd R_design,
																			int print_threshold){
	// Parse inputs:
	this->Nx = Nx;
	this->Nu = Nu;

	this->label_dim0 = label_dim0;
	this->label_dim1 = label_dim1;

	this->topic_lqr_gains = topic_lqr_gains;
	this->topic_lqr_gains_id = topic_lqr_gains_id;
	this->lqr_gains_id = lqr_gains_id;
	this->full_verbosity = full_verbosity;
	this->ack_received = false;

	int rows_lim = 0;
	int cols_lim = 0;

	// Call ros::init:
	std::string node_name("LQRGainsGenerator_node");
	int argc = 1; char name[] = "\0"; char* argv[1]; argv[0] = name;
	ros::init(argc, argv, node_name.c_str());

	// Initialize node:
	this->nh = std::make_shared<ros::NodeHandle>();

	// Initialize publisher:
	this->lqr_gains_pub = this->nh->advertise<userES_pubsub_lqr::Controller>(this->topic_lqr_gains,100);

	// Activate subscriber:
	this->lqr_gains_acknowledgment_sub = this->nh->subscribe(this->topic_lqr_gains_id,100,&LQRGainsGenerator::acknowledgement,this);

	// Fill in the layout of the request, as it will not change:
	size_t controller_dimensions = 2; // The controller is a plain object
	this->msg_controller.layout.dim = std::vector<std_msgs::MultiArrayDimension>(controller_dimensions);
	this->msg_controller.layout.dim[0].label 	= this->label_dim0;
	this->msg_controller.layout.dim[0].size 		= this->Nu;
	this->msg_controller.layout.dim[0].stride 	= this->Nu;
	this->msg_controller.layout.dim[1].label 	= this->label_dim1;
	this->msg_controller.layout.dim[1].size 		= this->Nx;
	this->msg_controller.layout.dim[1].stride 	= this->Nx * this->Nu; // Not needed
	this->msg_controller.layout.data_offset 		= 0;
	// this->msg_controller.do_experiment = 	do_experiment;
	this->msg_controller.full_verbosity = full_verbosity;
	this->Nel = this->Nx * this->Nu;

	// Read linear system from yaml file:
	this->A = A;
	this->B = B;
	this->N = N;

	if(this->full_verbosity){
		std::cout << "Linear system" << std::endl;
		std::cout << "=============" << std::endl;
		get_print_matrix_limits(A,print_threshold,rows_lim,cols_lim);
		std::cout << " A: "<< std::endl << this->A.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;

		get_print_matrix_limits(B,print_threshold,rows_lim,cols_lim);
		std::cout << " B: "<< std::endl << this->B.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;

		get_print_matrix_limits(N,print_threshold,rows_lim,cols_lim);
		std::cout << " N: "<< std::endl << this->N.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;
	}
	std::cout << "N.rows() = " << N.rows() << std::endl;
	std::cout << "N.cols() = " << N.cols() << std::endl;

	// Design the LQR controller:
	this->Q_design	= Q_design;
	this->R_design = R_design;

	// Verbosity:
	if(this->full_verbosity){
		std::cout << "Design weights" << std::endl;
		std::cout << "==============" << std::endl;
		std::cout << "diag(Q): " << std::endl << this->Q_design.transpose() << std::endl;
		std::cout << "diag(R): " << std::endl << this->R_design.transpose() << std::endl;
	}

	// Minimal realization system:
	this->minimal_realization_system();
}

void
LQRGainsGenerator::acknowledgement(const std_msgs::UInt8 id_receiver){

	if(this->lqr_gains_id == id_receiver.data)
		this->ack_received = true;

	std::cout << "Acknowledgement received!\n";

	return;
}

void
LQRGainsGenerator::run(){

	if(this->K.rows() != (int)this->Nu){
		std::string error("@CostFunctionClient::get_value - The number of control inputs Nu must match with those selected at compilation time.\nNu = " + std::to_string(this->Nu) + ", this->K.rows() = " + std::to_string(this->K.rows()));
		throw std::runtime_error(error);
	}

	if(this->K.cols() != (int)this->Nx){
		std::string error("@CostFunctionClient::get_value - The number of control inputs Nu must match with those selected at compilation time.\nNx = " + std::to_string(this->Nx) + ", this->K.cols() = " + std::to_string(this->K.cols()));
		throw std::runtime_error(error);
	}

	// Call LQR solver:
	// lqrSolver.compute(this->Q_design.asDiagonal(),this->R_design.asDiagonal(),this->A,this->B,this->K);
	lqrSolver.compute(this->Qm,this->Rm,this->Am,this->Bm,this->Km);
	this->K = this->Km * this->N.transpose();

	// Fill in the controller message:
	this->msg_controller.controller_gain = std::vector<double>(this->Nel);
	for(size_t i=0;i<this->Nu;++i){
		for(size_t j=0;j<this->Nx;++j){
			this->msg_controller.controller_gain[ this->Nx * i + j ] = this->K(i,j);
		}
	}

	if(this->full_verbosity){
		std::cout << "Publishing LQR gain matrix K\n";
		// std::cout << "K (th)  = " << this->K.block(0,0,17,17).diagonal().transpose() << std::endl;
		// std::cout << "K (thd) = " << this->K.block(0,23,17,17).diagonal().transpose() << std::endl;
		std::cout << "K = " << this->K.transpose() << std::endl;
	}

	// Wait for loop:
	size_t my_rate = 10;
	size_t time_wait4ack = 5;
	ros::Rate loop_rate(my_rate);
	int c = 0;
	while(ros::ok() && !this->ack_received){

		ros::spinOnce(); // Needed when there're callbacks attached to subscribers

		if(!this->ack_received){

			// Verbosity every 1 second:
			if(c % my_rate == 0)
				std::cout << "Waiting for acknowledgment in topic " << "\"" << this->topic_lqr_gains_id << "\", with id = " << this->lqr_gains_id << std::endl;

			this->lqr_gains_pub.publish(msg_controller);
			++c;
		}

		if(c > time_wait4ack*my_rate){
			std::cout << "Acknowledgement not received, but stopped anyways after " << time_wait4ack << " sec.\n";
			this->ack_received = true;
		}

		loop_rate.sleep();
	}

	return;
}

void
LQRGainsGenerator::minimal_realization_system(void){

	this->Am = this->N.transpose() * this->A * this->N;
	this->Bm = this->N.transpose() * this->B;
	// this->Cm = this->C * this->N; // Where this->C = eye(size(A,1))
	this->Qm = this->N.transpose() * this->Q_design.asDiagonal() * this->N;
	this->Rm = this->R_design.asDiagonal();

	return;
}

void LQRGainsGenerator::get_print_matrix_limits(Eigen::MatrixXd M,
																								int print_threshold,
																								int & rows_print_lim,
																								int & cols_print_lim){

	if(M.rows() > print_threshold)
		rows_print_lim = print_threshold;
	else
		rows_print_lim = M.rows();

	if(M.cols() > print_threshold)
		cols_print_lim = print_threshold;
	else
		cols_print_lim = M.cols();

	return;
}
