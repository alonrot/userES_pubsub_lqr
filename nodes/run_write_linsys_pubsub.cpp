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
#include "ReadYamlParameters.hpp"
#include <iostream>

int main (int argc, char const *argv[])
{

  if(argc != 3){
    throw std::runtime_error("Pass exactly 2 input arguments: Nx Nu");
  }

	int Nx = atoi(argv[1]);
	int Nu = atoi(argv[2]);
  // std::cout << "Nx = " << Nx < std::endl;

	Eigen::MatrixXd A(Nx,Nx);
	Eigen::MatrixXd B(Nx,Nu);

	A.setRandom();
	B.setRandom();

  // YAML::Emitter node_to_emit; // This corresponds to the lod API. Better not to use it
  YAML::Node node_to_write;
  node_to_write.SetStyle(YAML::EmitterStyle::Block);

  node_to_write["A"] = A;
  node_to_write["B"] = B;

  std::cout << "Path: " << YAML_CONFIG_PATH << std::endl;
  std::cout << "Note: This is defined in the CMakeLists.txt" << std::endl;

  // Write to file:
  std::ofstream fout(std::string(YAML_CONFIG_PATH) + "linear_model.yaml");
  fout << node_to_write;
  fout.close();

  return 0;
}