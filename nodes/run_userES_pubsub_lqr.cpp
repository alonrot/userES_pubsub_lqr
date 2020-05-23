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
// For Debugging:
    #include <chrono>
    #include <thread>

int main (int argc, char const *argv[])
{
  typedef std::vector<Eigen::VectorXd> Array2D;

  std::cout << "Loading input parameters" << std::endl;
  std::cout << "========================" << std::endl;
  std::cout << "Path: " << YAML_CONFIG_PATH << std::endl;
  std::cout << "Note: This is defined in the CMakeLists.txt" << std::endl;

  // Load parameters:
  ReadYamlParameters params = ReadYamlParameters(std::string(YAML_CONFIG_PATH) + "input_parameters.yaml"); // this is defined in the CMakeLists.txt
  
  // Standard options:
  size_t  Dim             = params.get<size_t>("Dim");
  size_t  Nrepresenters   = params.get<size_t>("Nrepresenters");
  size_t  Nsubsamples     = params.get<size_t>("Nsubsamples");
  size_t  T               = params.get<size_t>("T");
  size_t  Ndiv_plot       = params.get<size_t>("Ndiv_plot");
  size_t  Ndata_init      = params.get<size_t>("Ndata_init");
  size_t  Nwarm_starts    = params.get<size_t>("Nwarm_starts");
  size_t  MaxEvals        = params.get<size_t>("MaxEvals");
  size_t  Nline_searches  = params.get<size_t>("Nline_searches");
  double  xmin_s          = params.get<double>("xmin_s");
  double  xmax_s          = params.get<double>("xmax_s");
  bool    learn_hypers    = params.get<bool>("learn_hypers");
  std::string which_kernel    = params.get<std::string>("which_kernel");
  // std::string name_evalfun = params.get<std::string>("name_evalfun");
  std::string path2data_logging = params.get<std::string>("path2data_logging");

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
  size_t N_ind_Q, N_ind_R, Npar;
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

  // DBG flags:
  bool    write2file                                = params.get<bool>("write2file");
  bool    write2pyplot                              = params.get<bool>("write2pyplot");
  bool    write_matrices_to_file                    = params.get<bool>("write_matrices_to_file");
  bool    read_for_test_dH_MC_local_flag            = params.get<bool>("read_for_test_dH_MC_local_flag");
  bool    read_for_test_SampleBeliefLocations_flag  = params.get<bool>("read_for_test_SampleBeliefLocations_flag");
  bool    plot_true_function                        = params.get<bool>("plot_true_function");

  // Initialize the hyperparameters:
  Eigen::VectorXd lengthscale_s = params.get<Eigen::VectorXd>("lengthscale_s");
  size_t Nll = lengthscale_s.size();
  size_t hyperparam_dim = Nll + 1 + 1; // lengthscales + signal variance + measurement noise
  double  prior_std_s     = params.get<double>("prior_std_s");
  double  prior_std_n     = params.get<double>("prior_std_n");
  Eigen::VectorXd hyperparams(hyperparam_dim);
  hyperparams.head(Nll) = lengthscale_s.array().log();
  hyperparams(Nll)    = std::log(prior_std_s);
  hyperparams(Nll+1)  = std::log(prior_std_n);

  // Initialize the GP:
  libgp::GaussianProcess * gp = new libgp::GaussianProcess(Dim,which_kernel);
  gp->covf().set_loghyper(hyperparams);

  // Evaluation function:
  // std::string path2evalfun(std::string(YAML_CONFIG_PATH) + name_evalfun);
  // std::shared_ptr<CostFunction> cost_function = std::make_shared<CostFunctionFromPrior>(Dim,Ndiv_plot,path2evalfun);
  // std::shared_ptr<CostFunction> cost_function = std::make_shared<MyCostFunction>();
  // std::shared_ptr<CostFunction> cost_function = std::make_shared<CostFunctionClient_test>(service_name);
  std::shared_ptr<CostFunction> cost_function = std::make_shared<CostFunctionPubSub>(topic_controller,
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

  // Add data:
  if(Ndata_init > 0){
    Eigen::MatrixXd X(Ndata_init,Dim);
    Eigen::VectorXd foo(Dim);
    Eigen::VectorXd Y(Ndata_init);
    MathTools::sample_unif_mat(xmin_s,xmax_s,X);

    for(size_t i = 0; i < Ndata_init; ++i)
      Y(i) = cost_function->get_value(X.row(i));

    // Add to GP:
    for(size_t i = 0; i < Ndata_init; ++i) 
      gp->add_pattern(X.row(i).data(), Y(i));
  }

  // Path to plot:
  std::string path2data_logging_absolute(std::string(YAML_CONFIG_PATH) + "../" + path2data_logging);

  // Intialize input structure:
  std::shared_ptr<INSetup> in = std::make_shared<INSetup>();

  // Standard options:
  in->Dim  = Dim;
  in->Nrepresenters = Nrepresenters;
  in->Nsubsamples = Nsubsamples;
  in->T  = T;
  in->Ndiv_plot   = Ndiv_plot;
  in->Nwarm_starts = Nwarm_starts;
  in->MaxEval = MaxEvals;
  in->Nline_searches = Nline_searches;
  in->xmin_s = xmin_s;
  in->xmax_s = xmax_s;
  in->LearnHypers = learn_hypers;

  // DBG flags:
  in->write2file = write2file;
  in->write_matrices_to_file = write_matrices_to_file;
  in->read_for_test_dH_MC_local_flag = read_for_test_dH_MC_local_flag;
  in->read_for_test_SampleBeliefLocations_flag = read_for_test_SampleBeliefLocations_flag;

  // Plot with python:
  in->write2pyplot = write2pyplot;
  in->path2data_logging_absolute = path2data_logging_absolute;
  in->plot_true_function = plot_true_function;

  // Gaussian process:
  in->gp = gp;

  // Evaluation function:
  in->cost_function = cost_function;
  in->Nll = Nll;

  // Construct ES
  EntropySearch es(in);

  // Run ES:
  OutResults out_results;
  out_results = es.run();

  // Plotting stuff:
  size_t Nevals = out_results.in->gp->get_sampleset_size();
  Eigen::MatrixXd Xdata(Nevals,in->Dim);
  Eigen::VectorXd Ydata(Nevals);
  for(int i=0;i<Nevals;++i){
    Xdata.row(i) = out_results.in->gp->get_sampleset()->x(i);
    Ydata(i)     = out_results.in->gp->get_sampleset()->y(i);
  }

  std::cout << std::endl;
  std::cout << "    List of global minimums" << std::endl;
  std::cout << "    =======================" << std::endl;
  for(size_t i=0;i<Nevals;++i){
    std::cout << "     " << i+1 << ") [" << out_results.global_min_esti_x.row(i) << "]" << 
    ", mu(x_bg) = " << out_results.global_min_esti_mux(i) <<
    ", var(x_bg) = " << out_results.global_min_esti_varx(i) << std::endl;
  }

  std::cout << std::endl;
  std::cout << "    List of evaluations" << std::endl;
  std::cout << "    =======================" << std::endl;
  for(size_t i=0;i<Nevals;++i){
    std::cout << "     " << i+1 << ") X = [" << Xdata.row(i) << "]" << 
    ", y(X) = " << Ydata(i) << std::endl;
  }

}
