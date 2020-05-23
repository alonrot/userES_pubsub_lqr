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
#ifndef __LQR_GAINS_GENERATOR_H__
#define __LQR_GAINS_GENERATOR_H__

#include "ros/ros.h"
#include "userES_pubsub_lqr/Controller.h"
#include <memory>
#include <ct/optcon/optcon.h> // also includes ct_core
#include <Eigen/Dense>
#include <iostream>
#include "ReadYamlParameters.hpp"
#include "std_msgs/UInt8.h"
#include <string>

class LQRGainsGenerator {

public:
	LQRGainsGenerator(std::string topic_lqr_gains,
										std::string topic_lqr_gains_id,
										int lqr_gains_id,
										size_t Nx, size_t Nu,
										std::string label_dim0, std::string label_dim1,
										bool full_verbosity,
										Eigen::MatrixXd A, Eigen::MatrixXd B, 
										Eigen::MatrixXd N, 
										Eigen::VectorXd Q_empirical, Eigen::VectorXd R_empirical,
										int print_threshold);
	virtual ~LQRGainsGenerator(){}
	void acknowledgement(const std_msgs::UInt8 id_receiver);
	void run();
	void minimal_realization_system(void);
	void get_print_matrix_limits(	Eigen::MatrixXd M,
																int print_threshold,
																int & rows_print_lim,
																int & cols_print_lim);

private:
	std::shared_ptr<ros::NodeHandle> nh;
	userES_pubsub_lqr::Controller msg_controller;
	ros::Publisher 	lqr_gains_pub;
	ros::Subscriber lqr_gains_acknowledgment_sub;
	bool ack_received;
	std::string topic_lqr_gains;
	std::string topic_lqr_gains_id;
	int lqr_gains_id;
	size_t Nx, Nu, Nel;
	std::string label_dim0;
	std::string label_dim1;
	bool full_verbosity;
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd N;

	Eigen::MatrixXd Am;
	Eigen::MatrixXd Bm;
	Eigen::MatrixXd Qm;
	Eigen::MatrixXd Rm;

	Eigen::VectorXd Q_design;
	Eigen::VectorXd R_design;

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

};


#endif /* __LQR_GAINS_GENERATOR_H__ */