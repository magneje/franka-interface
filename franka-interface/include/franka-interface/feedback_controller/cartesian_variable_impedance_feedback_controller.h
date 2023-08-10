#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_VARIABLE_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_VARIABLE_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

// A Force-based Cartesian Variable Impedance Controller
class CartesianVariableImpedanceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  CartesianVariableImpedanceFeedbackControllerMessage cartesian_variable_impedance_feedback_params_;
  CartesianVariableImpedanceControllerSensorMessage cartesian_variable_impedance_sensor_msg_;
  
  const franka::Model *model_;
  bool use_commanded_damping_;
  bool use_commanded_mass_;
  bool use_inertia_shaping_ = 0;

  Eigen::Matrix<double, 6, 6> stiffness_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix<double, 6, 6> damping_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix<double, 6, 6> mass_ = Eigen::MatrixXd::Zero(6, 6);
  
  Eigen::Matrix<double, 6, 1> f_ext_initial_;
  Eigen::Matrix<double, 6, 1> alpha_, f_c_;
  Eigen::Matrix<double, 7, 1> tau_d_;
};

#endif // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_VARIABLE_IMPEDANCE_FEEDBACK_CONTROLLER_H_
