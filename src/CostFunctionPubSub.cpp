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
#include "CostFunctionPubSub.hpp"

// For Debugging:
    #include <chrono>
    #include <thread>

CostFunctionPubSub::CostFunctionPubSub(	std::string topic_controller, 
																				std::string topic_id_controller,
																				size_t id_controller,
																				std::string topic_cost_value, 
																				std::string topic_id_cost_value,
																				size_t id_cost_value,
																				std::string topic_robot_alive, 
																				size_t Nx, size_t Nu,
																				std::string label_dim0, std::string label_dim1,
																				bool full_verbosity,
																				Eigen::MatrixXd A, Eigen::MatrixXd B, 
																				Eigen::MatrixXd N, 
																				Eigen::VectorXd Q_empirical, 
																				Eigen::VectorXd R_empirical,
																				Array2D Q_parametrization, 
																				Array2D R_parametrization,
																				size_t N_ind_Q, size_t N_ind_R,
																				bool use_linear_scaling,
																				bool ask_user_for_value,
																				int print_threshold){

	int i;
	int rows_lim = 0;
	int cols_lim = 0;

	// Parse inputs:
	this->Nx = Nx;
	this->Nu = Nu;

	// Error checking:
	if(this->K.rows() != (int)this->Nu){
		throw std::runtime_error("@CostFunctionClient::get_value - The number of control inputs Nu must match with those selected at compilation time");
	}

	if(this->K.cols() != (int)this->Nx){
		throw std::runtime_error("@CostFunctionClient::get_value - The number of states Nx must match with those selected at compilation time");
	}

	this->label_dim0 = label_dim0;
	this->label_dim1 = label_dim1;

	this->cost_value = 0.0;
	this->cost_arrived = false;

	this->topic_controller = topic_controller; 
	this->topic_id_controller = topic_id_controller;
	this->id_controller = id_controller;
	this->topic_cost_value = topic_cost_value; 
	this->topic_id_cost_value = topic_id_cost_value;
	this->id_cost_value = id_cost_value;
	this->topic_robot_alive = topic_robot_alive; 

	this->topic_controller = topic_controller;
	this->topic_cost_value = topic_cost_value;

	this->use_linear_scaling = use_linear_scaling;

	this->ask_user_for_value = ask_user_for_value;

	// Call ros::init:
	std::string node_name("cost_function_node");
	int argc = 1; char name[] = "\0"; char* argv[1]; argv[0] = name;
	ros::init(argc, argv, node_name.c_str());

	// Initialize node:
	this->nh = std::make_shared<ros::NodeHandle>();

	// Communication of Controller:
		// Publish controller:
		this->controller_pub = this->nh->advertise<userES_pubsub_lqr::Controller>(this->topic_controller,100);
		// Subscribe to acknowlegment:
		this->controller_ack_sub = this->nh->subscribe(this->topic_id_controller,100,&CostFunctionPubSub::check_controller_acknowledgment,this);		
		// Controller message:
		size_t controller_dimensions = 2; // The controller is a plain object
		this->msg_controller.layout.dim = std::vector<std_msgs::MultiArrayDimension>(controller_dimensions);
		this->msg_controller.layout.dim[0].label 	= this->label_dim0;
		this->msg_controller.layout.dim[0].size 		= this->Nu;
		this->msg_controller.layout.dim[0].stride 	= this->Nu;
		this->msg_controller.layout.dim[1].label 	= this->label_dim1;
		this->msg_controller.layout.dim[1].size 		= this->Nx;
		this->msg_controller.layout.dim[1].stride 	= this->Nx * this->Nu; // Not needed
		this->msg_controller.layout.data_offset 		= 0;
		this->msg_controller.full_verbosity = full_verbosity;
		this->Nel = this->Nx * this->Nu;

	// Communication of Cost:
		// Subscribe to acknowlegment:
		this->cost_sub = this->nh->subscribe(this->topic_cost_value,10,&CostFunctionPubSub::check_cost_arrived,this);	
		// Publish controller:
		this->cost_ack_pub = this->nh->advertise<std_msgs::UInt8>(this->topic_id_cost_value,10);
		// Cost acknowledgment:
		this->msg_cost_ack.data = this->id_cost_value;

	// Activate subscriber:
	this->robot_alive_sub = this->nh->subscribe(this->topic_robot_alive,1,&CostFunctionPubSub::check_robot_alive,this);

	// Read linear system from yaml file:
	this->A = A;
	this->B = B;
	this->N = N;

	if(full_verbosity){
		std::cout << " Linear system" << std::endl;
		std::cout << " =============" << std::endl;

		get_print_matrix_limits(A,print_threshold,rows_lim,cols_lim);
		std::cout << " A: "<< std::endl << this->A.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;

		get_print_matrix_limits(B,print_threshold,rows_lim,cols_lim);
		std::cout << " B: "<< std::endl << this->B.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;

		get_print_matrix_limits(N,print_threshold,rows_lim,cols_lim);
		std::cout << " N: "<< std::endl << this->N.block(0,0,rows_lim,cols_lim) << std::endl << std::endl;
	}

	// Design the LQR controller:
	this->Q_empirical	= Q_empirical;
	this->R_empirical = R_empirical;

	if(full_verbosity){
		std::cout << " Empirical weights" << std::endl;
		std::cout << " ==============" << std::endl;
		std::cout << " Q_empirical: " << this->Q_empirical.transpose() << std::endl << std::endl;
		std::cout << " R_empirical: " << this->R_empirical.transpose() << std::endl << std::endl;
	}

	// Initialize the design matrices equal to the empirical ones that are used to compute the quadratic cost:
	this->Q_design 	= Q_empirical;
	this->R_design 	= R_empirical;

	this->Q_parametrization = Q_parametrization;
	this->R_parametrization = R_parametrization;

	this->N_ind_Q = N_ind_Q;
	this->N_ind_R = N_ind_R;

	this->Npars_per_row_of_Q = Eigen::VectorXd(this->N_ind_Q);
	this->Npars_per_row_of_R = Eigen::VectorXd(this->N_ind_R);

	for(i=0;i<this->N_ind_Q;++i){
		this->Npars_per_row_of_Q(i) = this->Q_parametrization[i].size() - 2;
		std::cout << "this->Npars_per_row_of_Q(i) = " << this->Npars_per_row_of_Q(i) << std::endl;
	}

	for(i=0;i<this->N_ind_R;++i){
		this->Npars_per_row_of_R(i) = this->R_parametrization[i].size() - 2;
		std::cout << "this->Npars_per_row_of_R(i) = " << this->Npars_per_row_of_R(i) << std::endl;
	}

	// Minimal realization system:
	this->minimal_realization_system();
}

void
CostFunctionPubSub::check_controller_acknowledgment(const std_msgs::UInt8 & msg_id_controller){

	if(msg_id_controller.data == this->id_controller){
		this->controller_received = true;
	}

	return;
}

void
CostFunctionPubSub::check_robot_alive(const std_msgs::Bool & msg_robot_alive){
	this->robot_is_alive = msg_robot_alive.data;
	return;
}

void
CostFunctionPubSub::check_cost_arrived(const userES_pubsub_lqr::CostValue & msg_cost_value){

	// If the experiment is done, we update some variables:
	this->cost_value = msg_cost_value.cost_experiment;
	this->cost_arrived = true;

	// Send acknowledgment:
	this->cost_ack_pub.publish(this->msg_cost_ack);

	return;
}

void CostFunctionPubSub::get_print_matrix_limits(Eigen::MatrixXd M,
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



double
CostFunctionPubSub::get_ii_linear(double theta, double a, double b){
  return a * theta + b;
}

double
CostFunctionPubSub::get_ii_10exp(double theta, double a, double b){
  return std::pow(10,a * theta + b);
}

void
CostFunctionPubSub::minimal_realization_system(void){

	this->Am = this->N.transpose() * this->A * this->N;
	this->Bm = this->N.transpose() * this->B;
	// this->Cm = this->C * this->N; // Where this->C = eye(size(A,1))
	this->Qm = this->N.transpose() * this->Q_design.asDiagonal() * this->N;
	this->Rm = this->R_design.asDiagonal();

	return;
}

double
CostFunctionPubSub::get_value(const Eigen::VectorXd pars){

	size_t i, j, c;
	size_t ii, jj, ind_col_a, ind_col_b;
  double q_ii, r_ii, a, b;

  if(this->ask_user_for_value){
  	double user_cost;
		std::cout << "    Enter cost: ";
		std::cin >> user_cost;
  	return user_cost;
  }

  // Replace the weights by the corresponding parameters:
  c = 0;
  if(this->N_ind_Q != 0){
    for(size_t i=0;i<this->N_ind_Q;++i){
    	ind_col_a = this->Npars_per_row_of_Q(i);
    	ind_col_b = this->Npars_per_row_of_Q(i)+1;
      a = this->Q_parametrization[i](ind_col_a);
      b = this->Q_parametrization[i](ind_col_b);

      if(this->use_linear_scaling)
	      q_ii = this->get_ii_linear(pars(c),a,b);
	    else
	    	q_ii = this->get_ii_10exp(pars(c),a,b);

      for(size_t j=0;j<this->Npars_per_row_of_Q(i);++j){
        jj = (size_t)this->Q_parametrization[i](j)-1;
        this->Q_design(jj) = q_ii;
      }
      ++c;
    }
  }

  if(this->N_ind_R != 0){
    for(size_t i=0;i<this->N_ind_R;++i){
    	ind_col_a = this->Npars_per_row_of_R(i);
    	ind_col_b = this->Npars_per_row_of_R(i)+1;
      a = this->R_parametrization[i](ind_col_a);
      b = this->R_parametrization[i](ind_col_b);

      if(this->use_linear_scaling)
				r_ii = this->get_ii_linear(pars(c),a,b);
	    else
	    	r_ii = this->get_ii_10exp(pars(c),a,b);

      for(size_t j=0;j<this->Npars_per_row_of_R(i);++j){
        jj = (size_t)this->R_parametrization[i](j)-1;
        this->R_design(jj) = r_ii;
      }
      ++c;
    }
  }

	// Verbosity:
	// if(this->full_verbosity){
		std::cout << "Design weights" << std::endl;
		std::cout << "==============" << std::endl;
		std::cout << "Q_design: "<< std::endl << this->Q_design.transpose() << std::endl;
		std::cout << "R_design: "<< std::endl << this->R_design.transpose() << std::endl;
		std::cout << "pars:" << std::endl << pars.transpose() << std::endl;
	// }

	// Compute minimal realization system:
	this->minimal_realization_system();

	// Call LQR solver:
	lqrSolver.compute(this->Qm,this->Rm,this->Am,this->Bm,this->Km);
	this->K = this->Km * this->N.transpose();

	// Fill in the controller message:
	this->msg_controller.controller_gain = std::vector<double>(this->Nel);
	for(i=0;i<this->Nu;++i){
		for(j=0;j<this->Nx;++j){
			this->msg_controller.controller_gain[ this->Nx * i + j ] = this->K(i,j);
		}
	}

	// Run communication:
	this->communication();

	return this->cost_value;

	// Fake non-zero mean:
	// return this->cost_value - 0.4;

	// Debuging:
	// return (rand() % 2001 -1000)/1000;
}

bool
CostFunctionPubSub::communication(void){

	// Loop rates:
	ros::Rate loop_rate_wait(0.5); // Hz
	ros::Rate loop_rate(1000); // Hz

	// Wait a bit before checking if the robot is alive:
	size_t count = 1;
	while(ros::ok()){

		if(count > 2)
			break;

		++count;

		loop_rate_wait.sleep();
	}

	// Loop before publishing until the robot is available for roll-outs:
	this->robot_is_alive = false;
	bool keep_checking_if_robot_alive = true;
	while(ros::ok() && keep_checking_if_robot_alive){

		// This is what makes sure the callbacks are listened at each iteration:
		ros::spinOnce();

		if(!this->robot_is_alive){
			ROS_WARN("I wanna send a controller, but the robot is dead. Looping until it comes back to life");
			ROS_WARN("Listening topic \"%s\"\n",this->topic_robot_alive.c_str());
		}
		else{
			ROS_INFO("Robot is alive");
			this->robot_is_alive = false; // This flag has to be falsed. Otherwise, if the robot dies, the
																		// callback won't get activated, and this->robot_is_alive will stay true
			keep_checking_if_robot_alive = false;
		}

		loop_rate_wait.sleep();
	}

	// No need to do a waitfor() here as soon as the subscriber polls/loops at a fast rate: 
	this->controller_pub.publish(msg_controller);

	ROS_INFO("Controller published in topic \"%s\"...",this->topic_controller.c_str());

	// Verbosity:
	if(this->full_verbosity){
		std::cout << "@CostFunctionPubSub: K = \n";
		// std::cout << K.block(0,0,8,8) << std::endl;
		std::cout << K << std::endl;
	}

	bool controller_received_local = false;
	size_t count_acknowledgment = 1;
	size_t wait_limit_ack_k = 5;
	while(ros::ok() && !controller_received_local){

		// This is what makes sure the callbacks are listened at each iteration
		this->robot_is_alive = false;
		ros::spinOnce();

		controller_received_local = this->controller_received;
		if(!controller_received_local){

			if(this->robot_is_alive){
				ROS_WARN("I am waiting for the controller acknowledgment in topic \"%s\", with id = %i...",this->topic_id_controller.c_str(),this->id_controller);
				if(count_acknowledgment > wait_limit_ack_k){
					ROS_ERROR("The controller acknowledgment was probably not catched after %i seconds.\nTell me the cost value:",wait_limit_ack_k*2);
					double emergency_cost_value = 0.0;
					std::cin >> emergency_cost_value;
					this->cost_value = emergency_cost_value;
					return true;
				}
				++count_acknowledgment;
			}
			else{
				ROS_ERROR("Robot died. No controller acknowledgment arrived. Controller is assumed unstable.\nTell me the cost value:");
				double emergency_cost_value = 0.0;
				std::cin >> emergency_cost_value;
				this->cost_value = emergency_cost_value;
				return true;
			}

		}
		else{
			ROS_INFO("Controller acknowledgment received!");
			this->controller_received = false;
			controller_received_local = true;
		}

		// loop_rate.sleep();
		loop_rate_wait.sleep();
	}

	// Loop before publishing until the robot is available for roll-outs:
	bool cost_arrived_local = false;
	while(ros::ok() && !cost_arrived_local){

		// This is what makes sure the callbacks are listened at each iteration:
		this->robot_is_alive = false;
		ros::spinOnce();

		cost_arrived_local = this->cost_arrived; // Acknowledgment is automatically sent inside the callback
		if(!cost_arrived_local){

			if(this->robot_is_alive){
				ROS_WARN("I am waiting for the cost in topic \"%s\"",this->topic_cost_value.c_str());
			}
			else{
				double emergency_cost_value = 0.0;
				ROS_ERROR("Robot died. No cost arrived. Controller is assumed unstable.\nTell me the cost value:");
				std::cin >> emergency_cost_value;
				this->cost_value = emergency_cost_value;
				return true;
			}
		}
		else{
			ROS_INFO("Cost received!");
			this->cost_arrived = false;
			cost_arrived_local = true;
		}

		loop_rate_wait.sleep();
	}

	return true;
}