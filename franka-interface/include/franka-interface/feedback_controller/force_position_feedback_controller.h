#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

// A Hybrid Force Position Controller
class ForcePositionFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  ForcePositionFeedbackControllerMessage force_position_feedback_params_;
  ForcePositionControllerSensorMessage force_position_sensor_msg_;
  const franka::Model *model_;

  double position_kp_, position_kd_, force_kp_, force_ki_;
  Eigen::Matrix<double, 6, 6> S_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::Matrix<double, 6, 6> Sp_= Eigen::MatrixXd::Zero(6, 6);

  Eigen::Matrix<double, 6, 1> xe_, fe_, xes_, fes_;
  Eigen::Matrix<double, 7, 1> q_es_, tau_es_, tau_p_, tau_f_, tau_task_, tau_d_;
  Eigen::Matrix<double, 7, 1> last_q_es_, total_tau_es_;

};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_