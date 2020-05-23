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
#ifndef __COST_FUNCTION_CLIENT_H__
#define __COST_FUNCTION_CLIENT_H__

#include "ros/ros.h"
#include "CostFunction.hpp"
#include "userES_pubsub_lqr/Controller.h"
#include "userES_pubsub_lqr/CostValue.h"
#include <memory>
#include <ct/optcon/optcon.h> // also includes ct_core
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

class CostFunctionPubSub : public CostFunction {

public:
typedef std::vector<Eigen::VectorXd> Array2D;
CostFunctionPubSub(	std::string topic_controller, 
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
																				Eigen::VectorXd Q_empirical, Eigen::VectorXd R_empirical, 
																				Array2D Q_parametrization, 
																				Array2D R_parametrization,
																				size_t N_ind_Q, size_t N_ind_R,
																				bool use_linear_scaling,
																				bool ask_user_for_value,
																				int print_threshold);

	virtual ~CostFunctionPubSub(){}
	double 	get_value(const Eigen::VectorXd x);
	void 	check_controller_acknowledgment(const std_msgs::UInt8 & msg_id_robot_rollout);
	void check_robot_alive(const std_msgs::Bool & msg_robot_alive);
	void 	check_cost_arrived(const userES_pubsub_lqr::CostValue & msg_cost_value);
	void get_print_matrix_limits(	Eigen::MatrixXd M,
																int print_threshold,
																int & rows_print_lim,
																int & cols_print_lim);
	double	get_ii_linear(double theta, double a, double b);
	double	get_ii_10exp(double theta, double a, double b);
	bool communication(void);
	void minimal_realization_system(void);


private:

	std::shared_ptr<ros::NodeHandle> nh;
	
	// Communication of Controller:
	userES_pubsub_lqr::Controller msg_controller;
	ros::Publisher 	controller_pub;
	ros::Subscriber controller_ack_sub;
	std::string topic_controller;
	std::string topic_id_controller;
	size_t id_controller;
	bool controller_received;

	// Communication of Cost:
	userES_pubsub_lqr::CostValue 	msg_cost_value;
	std_msgs::UInt8 msg_cost_ack;
	ros::Subscriber cost_sub;
	ros::Publisher 	cost_ack_pub;
	std::string topic_cost_value; 
	std::string topic_id_cost_value;
	size_t id_cost_value;
	bool cost_arrived;

	// Communication of robot:
	ros::Subscriber robot_alive_sub;
	std::string topic_robot_alive; 
	bool robot_is_alive;

	size_t Nx, Nu, Nel;
	std::string label_dim0;
	std::string label_dim1;
	bool full_verbosity;
	bool experiment_ready;
	bool use_linear_scaling;
	bool ask_user_for_value;
	double cost_value;
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd N;

	Eigen::MatrixXd Am;
	Eigen::MatrixXd Bm;
	Eigen::MatrixXd Qm;
	Eigen::MatrixXd Rm;

	Eigen::VectorXd Q_empirical;
	Eigen::VectorXd R_empirical;
	Eigen::VectorXd Q_design;
	Eigen::VectorXd R_design;
	size_t N_ind_Q;
	size_t N_ind_R;
	Array2D Q_parametrization;
	Array2D R_parametrization;
	Eigen::VectorXd Npars_per_row_of_Q;
	Eigen::VectorXd Npars_per_row_of_R;


	// LQR solver and gain depends on which robot and task we are using:
	#ifdef 	APOLLO_POLE_BALANCING
		ct::optcon::LQR<4,1> lqrSolver; // <states,inputs>
		ct::core::FeedbackMatrix<4, 1> Km; // <states,inputs>
	#elif 	HERMES_BALANCING
		ct::optcon::LQR<22,17> lqrSolver; // <states,inputs>
		ct::core::FeedbackMatrix<22, 17> Km; // <states,inputs>
		ct::core::FeedbackMatrix<46, 17> K; // <states,inputs>
	#else
		ct::optcon::LQR<4,1> lqrSolver; // <states,inputs>
		ct::core::FeedbackMatrix<4, 1> Km; // <states,inputs>
		ct::core::FeedbackMatrix<4, 1> K; // <states,inputs>
	#endif


	// // ROS Communication:
	// ros::Subscriber id_robot_rollout_sub;
	// size_t id_robot_rollout;
	// std::string topic_id_robot_rollout;

};

#endif /* __COST_FUNCTION_CLIENT_H__ */