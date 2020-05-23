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

double
get_ii_linear(double theta, double a, double b){
  return a * theta + b;
}

double
get_ii_10exp(double theta, double a, double b){
  return std::pow(10,a*theta+b);
}

int main(int argc, char **argv){

  int c;

  // Array2D is a type for arrays with a fixed number of rows, and variable number of columns in each row
  typedef std::vector<Eigen::VectorXd> Array2D;

  if(argc != 4)
    // throw std::runtime_error("Required exactly <topic_controller> <topic_id_controller> <id_controller> <is_real_robot>\nWhere is_real_robot can be:\n    [1] If it is the real robot\n    [0] If it is a simulation\n");
    throw std::runtime_error("Required exactly <topic_controller> <topic_id_controller> <id_controller>");

  std::string topic_controller(argv[1]);
  std::string topic_id_controller(argv[2]);
  int id_controller = atoi(argv[3]);

  // Input parameters:
  std::string path2_config_file(std::string(YAML_CONFIG_PATH) + "input_parameters.yaml");
  YAML::Node node_des_pars_input      = YAML::LoadFile(path2_config_file);
  std::string file_name_des_weights   = node_des_pars_input["file_name_des_weights"].as<std::string>();
  std::string file_name_linsys        = node_des_pars_input["file_name_linsys"].as<std::string>();
  size_t Nx                           = node_des_pars_input["Nx"].as<size_t>();
  size_t Nu                           = node_des_pars_input["Nu"].as<size_t>();
  size_t Dim                          = node_des_pars_input["Dim"].as<size_t>();
  std::string label_dim0              = node_des_pars_input["label_dim0"].as<std::string>();
  std::string label_dim1              = node_des_pars_input["label_dim1"].as<std::string>();
  
  bool full_verbosity                 = node_des_pars_input["full_verbosity"].as<bool>();
  int print_threshold                 = node_des_pars_input["print_threshold"].as<int>();

  // Design weights:
  YAML::Node node_des_weights = YAML::LoadFile(std::string(YAML_CONFIG_PATH) + file_name_des_weights);
  Eigen::VectorXd Q_empirical  = node_des_weights["Q_empirical"].as<Eigen::VectorXd>();
  Eigen::VectorXd R_empirical  = node_des_weights["R_empirical"].as<Eigen::VectorXd>();
  Array2D Q_design = node_des_weights["Q_design"].as<Array2D>();
  Array2D R_design = node_des_weights["R_design"].as<Array2D>();
  Eigen::VectorXd pars        = node_des_weights["pars"].as<Eigen::VectorXd>();
  bool use_linear_scaling     = node_des_weights["use_linear_scaling"].as<bool>();

  // Linear system:
  YAML::Node node_linsys = YAML::LoadFile(std::string(YAML_CONFIG_PATH) + file_name_linsys);
  Eigen::MatrixXd A      = node_linsys["A"].as<Eigen::MatrixXd>();
  Eigen::MatrixXd B      = node_linsys["B"].as<Eigen::MatrixXd>();
  Eigen::MatrixXd N;
  if(node_linsys["N"])
    N = node_linsys["N"].as<Eigen::MatrixXd>();
  else{
    N = Eigen::MatrixXd::Identity(Nx,Nx);
  }

  // Error checking:
  size_t N_ind_Q, N_ind_R, Npar;
  if(Q_design[0](0) == 0)
    N_ind_Q = 0;
  else
    N_ind_Q = Q_design.size();

  if(R_design[0](0) == 0)
    N_ind_R = 0;
  else
    N_ind_R = R_design.size();

  // Parametrize?
  Npar = N_ind_Q + N_ind_R;
  bool parametrize;
  if(Npar == 0){    
    parametrize = false;
    std::cout << "No parametrization required\n";
  }
  else if(pars.size() != (int)Npar){
    std::string error("Variable \"pars\" must be of size " + std::to_string(N_ind_Q) + " + " + std::to_string(N_ind_R));
    throw std::runtime_error(error);
  }
  else{
    parametrize = true;
  }

  Eigen::VectorXd Npars_per_row_of_Q;
  Eigen::VectorXd Npars_per_row_of_R;

  if(parametrize){

    if(N_ind_Q + N_ind_R > Dim)
      throw std::runtime_error("Q and R parametrized with a number of parameters larger than Dim \n           Dim defined in config/input_parameters.yaml");

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
  }

  // Replace the weights by the corresponding parameters:
  double q_ii, r_ii, a, b;
  size_t jj;
  if(parametrize){
    c = 0;
    if(N_ind_Q != 0){
      for(size_t i=0;i<N_ind_Q;++i){
        a = Q_design[i](Npars_per_row_of_Q(i));
        b = Q_design[i](Npars_per_row_of_Q(i)+1);

        if(use_linear_scaling)
          q_ii = get_ii_linear(pars(c),a,b);
        else
          q_ii = get_ii_10exp(pars(c),a,b);

        for(size_t j=0;j<Npars_per_row_of_Q(i);++j){
          jj = (size_t)Q_design[i](j)-1;
          Q_empirical(jj) = q_ii;
        }
        ++c;
      }
    }
    if(N_ind_R != 0){
      for(size_t i=0;i<N_ind_R;++i){
        a = R_design[i](Npars_per_row_of_R(i));
        b = R_design[i](Npars_per_row_of_R(i)+1);

        if(use_linear_scaling)
          r_ii = get_ii_linear(pars(c),a,b);
        else
          r_ii = get_ii_10exp(pars(c),a,b);

        for(size_t j=0;j<Npars_per_row_of_R(i);++j){
          jj = (size_t)R_design[i](j)-1;
          R_empirical(jj) = r_ii;
        }
        ++c;
      }
    }
  }

  std::cout << "pars = " << pars.transpose() << std::endl;


	LQRGainsGenerator lqr_gains_generator = LQRGainsGenerator(topic_controller,
                                                            topic_id_controller,
                                                            id_controller,
                                                            Nx,Nu,
                                                            label_dim0,label_dim1,
                                                            full_verbosity,
                                                            A,B,N,
                                                            Q_empirical,R_empirical,
                                                            print_threshold);

	lqr_gains_generator.run();

	return 0;
}