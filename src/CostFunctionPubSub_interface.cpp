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
#include "EntropySearch.hpp" // This refers to the EntropySearchLib
#include "CostFunctionPubSub.hpp" // This is a customized user function that inherits the CostFunction class
#include <string>

extern "C"
{
	// Globally defined:
  CostFunction * cost_function;

  // Construct:
  void cost_function_init(void){

	  typedef std::vector<Eigen::VectorXd> Array2D;

	  std::cout << "Loading input parameters" << std::endl;
	  std::cout << "========================" << std::endl;
	  std::cout << "Path: " << YAML_CONFIG_PATH << std::endl;
	  std::cout << "Note: This is defined in the CMakeLists.txt" << std::endl;

	  // Load parameters:
	  ReadYamlParameters params = ReadYamlParameters(std::string(YAML_CONFIG_PATH) + "input_parameters_mESCO.yaml"); // this is defined in the CMakeLists.txt
	  
	  // Standard options:
	  size_t  Dim             = params.get<size_t>("Dim");

	  // Variables for the client that requires a function evaluation upon request:
	  std::string topic_controller    = params.get<std::string>("topic_controller");
	  std::string topic_id_controller = params.get<std::string>("topic_id_controller");
	  size_t id_controller         = params.get<size_t>("id_controller");

	  std::string topic_cost_value    = params.get<std::string>("topic_cost_value");
	  std::string topic_id_cost_value    = params.get<std::string>("topic_id_cost_value");
	  size_t id_cost_value         = params.get<size_t>("id_cost_value");

	  std::string topic_robot_alive    = params.get<std::string>("topic_robot_alive");
	  bool ask_user_for_value             = params.get<bool>("ask_user_for_value");

	  // Input parameters:
	  std::string file_name_des_weights   = params.get<std::string>("file_name_des_weights");
	  std::string file_name_linsys        = params.get<std::string>("file_name_linsys");
	  size_t Nx                           = params.get<size_t>("Nx");
	  size_t Nu                           = params.get<size_t>("Nu");
	  std::string label_dim0              = params.get<std::string>("label_dim0");
	  std::string label_dim1              = params.get<std::string>("label_dim1");
	  
	  bool full_verbosity                 = params.get<bool>("full_verbosity");
	  int print_threshold                 = params.get<int>("print_threshold");


	  // Design weights:
	  YAML::Node node_des_weights = YAML::LoadFile(std::string(YAML_CONFIG_PATH) + file_name_des_weights);
	  Eigen::VectorXd Q_empirical  = node_des_weights["Q_empirical"].as<Eigen::VectorXd>();
	  Eigen::VectorXd R_empirical  = node_des_weights["R_empirical"].as<Eigen::VectorXd>();
	  Array2D Q_design = node_des_weights["Q_design"].as<Array2D>();
	  Array2D R_design = node_des_weights["R_design"].as<Array2D>();
	  bool use_linear_scaling = node_des_weights["use_linear_scaling"].as<bool>();

	  // Linear system:
	  YAML::Node node_linsys        = YAML::LoadFile(std::string(YAML_CONFIG_PATH) + file_name_linsys);
	  Eigen::MatrixXd A             = node_linsys["A"].as<Eigen::MatrixXd>();
	  Eigen::MatrixXd B             = node_linsys["B"].as<Eigen::MatrixXd>();
	  Eigen::MatrixXd N;
	  if(node_linsys["N"])
	    N = node_linsys["N"].as<Eigen::MatrixXd>();
	  else{
	    N = Eigen::MatrixXd::Identity(Nx,Nx);
	  }

	  // Error checking:
	  size_t N_ind_Q, N_ind_R;
	  if(Q_design[0](0) == 0)
	    N_ind_Q = 0;
	  else
	    N_ind_Q = Q_design.size();

	  if(R_design[0](0) == 0)
	    N_ind_R = 0;
	  else
	    N_ind_R = R_design.size();

	  Eigen::VectorXd Npars_per_row_of_Q;
	  Eigen::VectorXd Npars_per_row_of_R;

	  if(N_ind_Q + N_ind_R != Dim)
	    throw std::runtime_error("Q and R parametrized with a number of parameters different than Dim \n           Dim defined in config/input_parameters.yaml");

	  if(Q_empirical.size() != (int)Nx)
	    throw std::runtime_error("Q_empirical must be of size Nx");

	  if(R_empirical.size() != (int)Nu)
	    throw std::runtime_error("R_empirical must be of size Nx");

	  // Get first parameters: // TODO: this assumes that parameters go in pairs of two. Has to be generalized to different Nr. of columns per row
	  Npars_per_row_of_Q = Eigen::VectorXd(N_ind_Q);
	  Npars_per_row_of_R = Eigen::VectorXd(N_ind_R);

	  for(size_t i=0;i<N_ind_Q;++i){

	    if(Q_design[i].size() < 3){
	      std::string my_error("Q_design.row(" + std::to_string(i) + ").size() < 3");
	      throw std::runtime_error(my_error);
	    }

	    Npars_per_row_of_Q(i) = Q_design[i].size() - 2;
	    for(size_t j=0;j<Npars_per_row_of_Q(i);++j){
	      if(Q_design[i](j) > (int)Nx)
	        throw std::runtime_error("Indices in Q_design.row(" + std::to_string(i) + ").col(" + std::to_string(j) + ") cannot be larger than Nx (One-start indexing type)\n           Nx defined in config/input_parameters.yaml");

	      if(Q_design[i](j) < 0)
	        throw std::runtime_error("Indices in Q_design.row(" + std::to_string(i) + ").col(" + std::to_string(j) + ") cannot be < 0 (One-start indexing type)");
	    }
	  }

	  for(size_t i=0;i<N_ind_R;++i){
	    
	    if(R_design[i].size() < 3){
	      std::string my_error("R_design.row(" + std::to_string(i) + ").size() < 3");
	      throw std::runtime_error(my_error);
	    }

	    Npars_per_row_of_R(i) = R_design[i].size() - 2;
	    for(size_t j=0;j<Npars_per_row_of_R(i);++j){
	      if(R_design[i](j) > (int)Nu)
	        throw std::runtime_error("Indices in R_design.row(" + std::to_string(i) + ").col(" + std::to_string(j) + ") cannot be larger than Nx (One-start indexing type)\n           Nx defined in config/input_parameters.yaml");

	      if(R_design[i](j) < 0)
	        throw std::runtime_error("Indices in R_design.row(" + std::to_string(i) + ").col(" + std::to_string(j) + ") cannot be < 0 (One-start indexing type)");
	    }

	  }

    cost_function = new CostFunctionPubSub(topic_controller,
                                          topic_id_controller,
                                          id_controller,
                                          topic_cost_value,
                                          topic_id_cost_value,
                                          id_cost_value,
                                          topic_robot_alive,
                                          Nx,Nu,
                                          label_dim0,label_dim1,
                                          full_verbosity,
                                          A,B,N,
                                          Q_empirical,R_empirical,
                                          Q_design,R_design,
                                          N_ind_Q,N_ind_R,
                                          use_linear_scaling,
                                          ask_user_for_value,
                                          print_threshold);
	}

	double cost_function_call(double * pars, int Dim){

		Eigen::VectorXd pars_eig = Eigen::VectorXd(Dim);
		for(int i=0;i<Dim;++i){
			pars_eig(i) = pars[i];
		}

		return cost_function->get_value(pars_eig);

	}


}