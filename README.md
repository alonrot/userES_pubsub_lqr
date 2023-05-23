Description
=========
ROS communication framework between a Bayesian optimization (BO) algorithm and a robot, to automatically tune a robot controller. The aim of this tool is to robustly pass messages between BO and the robot. In a nutshell, when the BO algorithm requests a new evaluation, this framework triggers a new experiment on the robot. BO waits until the experiment is finished to receive the performance value. This tool robustifies many technical issues that usually take in robot learning. For example, it can handle:
* Unstable controllers
* Resetting the robot
* The optimizer crashing

This tool has been used in the project to which the following publication is associated

  Alonso Marco, Philipp Hennig, Jeannette Bohg, Stefan Schaal, Sebastian Trimpe,
  "Automatic LQR Tuning Based on Gaussian Process Global Optimization", 
  IEEE International Conference on Robotics and Automation (ICRA),
  2016, accepted.

Therein, BO was used to automatically tune parameters of a whole-body Linear Quadratic Regulator (LQR) for a two-legged humanoid robot to perform a squatting task. Also, BO was used to automatically tune an LQR controller to balance an inverted pendulum with a robot arm. See the associated publication [here](https://arxiv.org/abs/1605.01950) and a video description of the method and the results for the [robot arm](https://youtu.be/TrGc4qp3pDM) and for the [two-legged humanoid](https://youtu.be/udJAK60IWEc).

This project uses a reimplementation of [Entropy Search (ES)](http://www.jmlr.org/papers/volume13/hennig12a/hennig12a.pdf) in `c++`, which code can be found [here](https://github.com/alonrot/EntropySearchCpp). The original Matlab code can be found [here](https://github.com/ProbabilisticNumerics/entropy-search). An alternative version that improves on efficiency of the original ES code and provides several plotting tools can be found [here](https://github.com/alonrot/userES).

	Philipp Hennig and Christian J Schuler,
	"Entropy Search for Information-Efficient Global Optimization", 
	The Journal of Machine Learning Research (JMLR),
	2012, accepted.


Requirements
============
This package requires:
* [ROS](https://www.ros.org/): Set of software libraries and tools to build robot applications.
* [yaml-cpp](https://codedocs.xyz/jbeder/yaml-cpp.svg): [YAML 1.2 spec](http://www.yaml.org/spec/1.2/spec.html) parser and emitter in C++.
* [cmake](http://www.cmake.org/): cross-platform, open-source build system
* [Eigen3](http://eigen.tuxfamily.org/): template library for linear algebra
* [ct](https://arxiv.org/abs/1801.04290): The Control Toolbox - Library for Robotics, Optimal and Model Predictive Control

Contact information
===================
For any questions, please, send an e-mail to: 

   marcovalle.a(at)gmail.com

