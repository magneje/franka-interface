//
// Created by magneje on 11/14/22.
//
#include "franka-interface/feedback_controller/cartesian_variable_impedance_feedback_controller.h"

#include <exception>

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

void CartesianVariableImpedanceFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = cartesian_variable_impedance_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    for (int i = 0; i < 6; i++) {
      stiffness_(i, i) = cartesian_variable_impedance_feedback_params_.stiffness(i);
    }
    
    use_commanded_damping_ = cartesian_variable_impedance_feedback_params_.use_commanded_damping();
    if (use_commanded_damping_) {
      for (int i = 0; i < 6; i++) {
        damping_(i, i) = cartesian_variable_impedance_feedback_params_.damping(i);
      }
    } else {
      for (int i = 0; i < 6; i++) {
        damping_(i, i) = 2. * sqrt(stiffness_(i, i));
      }
    }
    
    use_commanded_mass_ = cartesian_variable_impedance_feedback_params_.use_commanded_mass();
    if (use_commanded_mass_) {
      for (int i = 0; i < 6; i++) {
        mass_(i, i) = cartesian_variable_impedance_feedback_params_.mass(i);
      }
    } else {
      for (int i = 0; i < 6; i++) {
        mass_(i, i) = 1;
      }
    }
  } else {
    std::cout << "Parsing CartesianVariableImpedanceFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void CartesianVariableImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void CartesianVariableImpedanceFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(cartesian_variable_impedance_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 6; i++) {
      stiffness_(i, i) = cartesian_variable_impedance_sensor_msg_.stiffness(i);
    }
    if (use_commanded_damping_) {
      for (int i = 0; i < 6; i++) {
        damping_(i, i) = cartesian_variable_impedance_sensor_msg_.damping(i);
      }
    } else {
      for (int i = 0; i < 6; i++) {
        damping_(i, i) = 2. * sqrt(stiffness_(i, i));
      }
    }
    if (use_commanded_mass_) {
      for (int i = 0; i < 6; i++) {
        mass_(i, i) = cartesian_variable_impedance_sensor_msg_.mass(i);
      }
    } else {
      for (int i = 0; i < 6; i++) {
        mass_(i, i) = 1;
      }
    }
  }
  if (f_ext_initialized_ == 0) {
    f_ext_initial_(robot_state.O_F_ext_hat_K.data());
    f_ext_initialized_ = 1;
  }
}

void CartesianVariableImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                         TrajectoryGenerator *traj_generator) {
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  std::array<double, 49> inertia_array = model_->mass(robot_state);
  std::array<double, 6> force_array = robot_state.O_F_ext_hat_K; // FIXME: this resulted in an error earlier.
  
  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7> > inertia(inertia_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::VectorXd> force(force_array.data(), 6);
  Eigen::Matrix<double, 6, 1> dx(jacobian * dq);
  Eigen::Matrix<double, 7, 7> inertia_inv(inertia.inverse());
  Eigen::Matrix<double, 7, 6> jacobian_transpose(jacobian.transpose());
  Eigen::Matrix<double, 6, 6> J_times_M_inv_times_J_transpose(jacobian*inertia_inv*jacobian_transpose);
  Eigen::Matrix<double, 6, 6> inertia_cart(J_times_M_inv_times_J_transpose.inverse());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  
  //force.tail(3) << Eigen::MatrixXd::Zero(3, 1);
  
  
  //*************************
  // Print all matrices (and calculate by hand after)
  // Inertia
  /*std::cout << "Inertia H(q):\n   [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << inertia(row_i, col_j) << ", ";
    }
    std::cout << ";\n    ";
  }
  std::cout << "]\n";*/
  
  // Inverse of inertia
  /*std::cout << "Inertia inverse:\n   [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << inertia_inv(row_i, col_j) << ", ";
    }
    std::cout << ";\n    ";
  }
  std::cout << "]\n";*/
  
  // Jacobian
  /*std::cout << "Jacobian J(q): [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 7; col_j++) {
      std::cout << jacobian(row_i, col_j) << ", ";
    }
    std::cout << ";\n                ";
  }
  std::cout << "]\n";*/
  
  // Transpose of jacobian
  /*std::cout << "J^T: [";
  for(int row_i = 0; row_i < 7; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << jacobian_transpose(row_i, col_j) << ", ";
    }
    std::cout << ";\n                ";
  }
  std::cout << "]\n";*/
  
  // J * inv(H(q)) * J^T
  /*std::cout << "J * inv(H(q)) * J^T:\n   [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << J_times_M_inv_times_J_transpose(row_i, col_j) << ", ";
    }
    std::cout << ";\n    ";
  }
  std::cout << "]\n";*/
  
  // Lambda(q) = inv(J * inv(H(q)) * J^T)
  /*std::cout << "Task space robot inertia Lambda(q): [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << inertia_cart(row_i, col_j) << ", ";
    }
    std::cout << ";\n                                   ";
  }
  std::cout << "]\n";*/
  
  // Sanity check
  Eigen::Matrix<double, 6, 6> eye = J_times_M_inv_times_J_transpose*inertia_cart;
  std::cout << "Identity matrix (from inertia cart times inverse):\n   [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << eye(row_i, col_j) << ", ";
    }
    std::cout << ";\n    ";
  }
  std::cout << "]\n";
  //*************************

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator);

  if (pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  Eigen::Vector3d position_d(pose_trajectory_generator->get_desired_position());
  Eigen::Quaterniond orientation_d(pose_trajectory_generator->get_desired_orientation());

  // compute error to desired equilibrium pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d;

  // orientation error
  // "difference" quaternion
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  
  // TODO: add desired force (might require different traj-generator??)
  
  // Check if force_array shows total force or external force only
  //std::cout << "Force array: [" << force_array[0] << ", " << force_array[1] << ", " << force_array[2] << ", " << force_array[3] << ", " << force_array[4] << ", " << force_array[5] << "]\n";
  
  //std::cout << "F_ext_initial: [" << f_ext_initial_[0] << ", " << f_ext_initial_[1] << ", " << f_ext_initial_[2] << ", " << f_ext_initial_[3] << ", " << f_ext_initial_[4] << ", " << f_ext_initial_[5] << "]\n";
  
  // Assume that desired velocity and acceleration is 0
  Eigen::Matrix<double, 6, 1> dx_d = Eigen::MatrixXd::Zero(6, 1);
  Eigen::Matrix<double, 6, 1> ddx_d = Eigen::MatrixXd::Zero(6, 1);
  Eigen::Matrix<double, 6, 1> dx_error = dx - dx_d;
  
  // Check joint inertia
  /*std::cout << "Jacobian J(q): [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 7; col_j++) {
      std::cout << jacobian(row_i, col_j) << ", ";
    }
    std::cout << ";\n                ";
  }
  std::cout << "]\n";*/
  
  // Check cartesian inertia 
  /*std::cout << "Task space robot inertia Lambda(q): [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << inertia_cart(row_i, col_j) << ", ";
    }
    std::cout << ";\n                                   ";
  }
  std::cout << "]\n";*/
  
  // Implement VIC
  Eigen::Matrix<double, 6, 6> inertia_shaping = inertia_cart*mass_.inverse(); // equal to identity when no inertia shaping
  
  // Check inertia-shaping 
  /*std::cout << "Inertia shaping Lambda(q)*M-inv: [";
  for(int row_i = 0; row_i < 6; row_i++) {
    for(int col_j = 0; col_j < 6; col_j++) {
      std::cout << inertia_shaping(row_i, col_j) << ", ";
    }
    std::cout << ";\n                                ";
  }
  std::cout << "]\n";*/
  
  
  // f_c_ = inertia_shaping*(- stiffness_ * error - damping_ * dx_error) - (Eigen::MatrixXd::Identity(6,6) - inertia_shaping) * force;
  f_c_ = - stiffness_ * error - damping_ * dx_error;

  // compute control input
  tau_d_ = jacobian.transpose()*f_c_ + coriolis;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d_;
}
