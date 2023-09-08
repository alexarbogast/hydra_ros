#include <hydra_gazebo/positioner_hw_sim.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <boost/algorithm/clamp.hpp>


namespace hydra_gazebo {

bool PositionerHWSim::initSim(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              gazebo::physics::ModelPtr parent,
                              const urdf::Model* const urdf,
                              std::vector<transmission_interface::TransmissionInfo> transmissions) {
    model_nh.param<std::string>("arm_id", this->arm_id_, robot_namespace);
    if (this->arm_id_ != robot_namespace) {
    ROS_WARN_STREAM_NAMED(
        "positioner_hw_sim",
        "Caution: Positioner name differs! Read 'arm_id: "
            << this->arm_id_ << "' from parameter server but URDF defines '<robotNamespace>"
            << robot_namespace << "</robotNamespace>'. Will use '" << this->arm_id_ << "'!");
    }

    this->robot_ = parent;
    this->robot_initialized_ = false;

    this->robot_initialized_pub_ = model_nh.advertise<std_msgs::Bool>("initialized", 1);
    std_msgs::Bool msg;
    msg.data = static_cast<decltype(msg.data)>(false);
    this->robot_initialized_pub_.publish(msg);

    model_nh.param<double>("tau_ext_lowpass_filter", this->tau_ext_lowpass_filter_,
                         kDefaultTauExtLowpassFilter);

    for (const auto& transmission : transmissions) {
        if (transmission.type_ != "transmission_interface/SimpleTransmission") {
            continue;
        }
        if (transmission.joints_.empty()) {
            ROS_WARN_STREAM_NAMED("positioner_hw_sim",
                            "Transmission " << transmission.name_ << " has no associated joints.");
            return false;
        }
        if (transmission.joints_.size() > 1) {
            ROS_WARN_STREAM_NAMED(
                "positioner_hw_sim",
                "Transmission "
                    << transmission.name_
                    << " has more than one joint. Currently the za robot hardware simulation "
                    << " interface only supports one.");
            return false;
        }

        // Fill a 'Joint' struct which holds all necessary data
        auto joint = std::make_shared<za_gazebo::Joint>();
        joint->name = transmission.joints_[0].name_;
        if (urdf == nullptr) {
            ROS_ERROR_STREAM_NAMED(
                "positioner_hw_sim", "Could not find any URDF model. Was it loaded on the parameter server?");
            return false;
        }
        auto urdf_joint = urdf->getJoint(joint->name);
        if (not urdf_joint) {
            ROS_ERROR_STREAM_NAMED("positioner_hw_sim",
                "Could not get joint '" << joint->name << "' from URDF");
            return false;
        }
        joint->type = urdf_joint->type;
        joint_limits_interface::getJointLimits(urdf_joint, joint->limits);
        joint->axis = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);

        // Get a handle to the underlying Gazebo Joint
        gazebo::physics::JointPtr handle = parent->GetJoint(joint->name);
        if (not handle) {
            ROS_ERROR_STREAM_NAMED("positioner_hw_sim", "This robot has a joint named '"
                                                      << joint->name
                                                      << "' which is not in the gazebo model.");
            return false;
        }
        joint->handle = handle;
        this->joints_.emplace(joint->name, joint);
    }
    for (auto& pair : this->joints_) {
        initJointStateHandle(pair.second);
    }

    // register all supported command interfaces
    for (const auto& transmission : transmissions) {
        for (const auto& k_interface : transmission.joints_[0].hardware_interfaces_) {
            auto joint = this->joints_[transmission.joints_[0].name_];
            if (transmission.type_ == "transmission_interface/SimpleTransmission") {
                ROS_INFO_STREAM_NAMED("positioner_hw_sim", "Found transmission interface of joint '"
                                                   << joint->name << "': " << k_interface);
                if (k_interface == "hardware_interface/EffortJointInterface") {
                    initEffortCommandHandle(joint);
                    continue;
                }
                if (k_interface == "hardware_interface/PositionJointInterface") {
                    // Initiate position motion generator (PID controller)
                    joint->position_controller.initParam(model_nh.getNamespace() +
                                                         "/motion_generators/position/gains/" + joint->name);
                    initPositionCommandHandle(joint);
                    continue;
                }
                if (k_interface == "hardware_interface/VelocityJointInterface") {
                    // Initiate velocity motion generator (PID controller)
                    joint->velocity_controller.initParam(model_nh.getNamespace() +
                                                         "/motion_generators/velocity/gains/" + joint->name);
                    initVelocityCommandHandle(joint);
                    continue;
                }
            }

            if (transmission.type_ == "hydra_hw/PositionerStateInterface") {
                ROS_INFO_STREAM_NAMED("positioner_hw_sim",
                              "Found transmission interface '" << transmission.type_ << "'");
                try {
                    initPositionerStateHandle(this->arm_id_, *urdf, transmission);
                    continue;

                    } catch (const std::invalid_argument& e) {
                        ROS_ERROR_STREAM_NAMED("positioner_hw_sim", e.what());
                        return false;
                    }
            }

            ROS_WARN_STREAM_NAMED("positioner_hw_sim", "Unsupported transmission interface of joint '"
                                                 << joint->name << "': " << k_interface);
        }
    }

    // After all handles have been assigned to interfaces, register them
    registerInterface(&this->eji_);
    registerInterface(&this->pji_);
    registerInterface(&this->vji_);
    registerInterface(&this->jsi_);
    registerInterface(&this->psi_);

    initServices(model_nh);
    verifier_ = std::make_unique<za_gazebo::ControllerVerifier>(joints_, arm_id_);
    return true;
}

void PositionerHWSim::initJointStateHandle(const std::shared_ptr<za_gazebo::Joint>& joint) {
    this->jsi_.registerHandle(
        hardware_interface::JointStateHandle(joint->name, &joint->position,
                                            &joint->velocity, &joint->effort));
}

void PositionerHWSim::initEffortCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint) {
    this->eji_.registerHandle(
        hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->command));
}

void PositionerHWSim::initVelocityCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint) {
    this->vji_.registerHandle(
        hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->desired_velocity));
}

void PositionerHWSim::initPositionCommandHandle(const std::shared_ptr<za_gazebo::Joint>& joint) {
    this->pji_.registerHandle(
        hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->desired_position));
}

void PositionerHWSim::initPositionerStateHandle(
                               const std::string& robot,
                               const urdf::Model& urdf,
                               const transmission_interface::TransmissionInfo& transmission) {
    // Check if all joints defined in the <transmission> actually exist in the URDF
    for (const auto& joint : transmission.joints_) {
        if (not urdf.getJoint(joint.name_)) {
            throw std::invalid_argument("Cannot create hydra_hw/PositionerStateInterface for robot '" +
                                        robot + "_robot' because the specified joint '" + joint.name_ +
                                        "' in the <transmission> tag cannot be found in the URDF");
      }
      ROS_DEBUG_STREAM_NAMED("positioner_hw_sim",
                             "Found joint " << joint.name_ << " to belong to a positioner");
    }
    this->psi_.registerHandle(hydra_hw::PositionerStateHandle(robot + "_robot", this->robot_state_));
}

void PositionerHWSim::initServices(ros::NodeHandle& nh) {
    this->service_user_stop_ =
        nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
            "hydra_control/set_user_stop", [&](auto& request, auto& response) {
                //this->sm_.process_event(UserStop{static_cast<bool>(request.data)});
                response.success = true;
                return true;
            });
    this->service_controller_list_ = nh.serviceClient<controller_manager_msgs::ListControllers>(
      "controller_manager/list_controllers");
    this->service_controller_switch_ = nh.serviceClient<controller_manager_msgs::SwitchController>(
        "controller_manager/switch_controller");
}

void PositionerHWSim::readSim(ros::Time time, ros::Duration period) {
    for (const auto& pair : this->joints_) {
        auto joint = pair.second;
        joint->update(period);
    }
    this->updatePositionerState(time);
}

void PositionerHWSim::writeSim(ros::Time /*time*/, ros::Duration period) {
    // add state machine case

    for (auto& pair : this->joints_) {
        auto joint = pair.second;
        double effort = 0;
        if (joint->control_method == za_gazebo::ControlMethod::POSITION) {
            effort = positionControl(*joint, joint->desired_position, period);
        }
        else if (joint->control_method == za_gazebo::ControlMethod::VELOCITY) {
            //effort = velocityControl(*joint, joint->desired_velocity, period);
            joint->handle->SetVelocity(0, joint->desired_velocity);
        } 
        else if (joint->control_method == za_gazebo::ControlMethod::EFFORT) {
            effort = joint->command;
        }

        if (not std::isfinite(effort)) {
                ROS_WARN_STREAM_NAMED("positioner_hw_sim",
                                "Command for " << joint->name << "is not finite, won't send to robot");
                return;
        }
        joint->handle->SetForce(0, effort);
    }
}

double PositionerHWSim::positionControl(za_gazebo::Joint& joint, double setpoint, const ros::Duration& period) {
    double error;
    const double kJointLowerLimit = joint.limits.min_position;
    const double kJointUpperLimit = joint.limits.max_position;
    switch (joint.type)
    {
        case urdf::Joint::CONTINUOUS:
            angles::shortest_angular_distance_with_limits(joint.position, setpoint, kJointLowerLimit,
                                                    kJointUpperLimit, error);
            break;
        default:
            std::string error_message =
                "Only continuous joints are allowed for position control right now";
            ROS_FATAL("%s", error_message.c_str());
            throw std::invalid_argument(error_message);
    }

    return boost::algorithm::clamp(joint.position_controller.computeCommand(error, period), 
                                  -joint.limits.max_effort, joint.limits.max_effort);

}

double PositionerHWSim::velocityControl(za_gazebo::Joint& joint, double setpoint, const ros::Duration& period) {
    return boost::algorithm::clamp(
        joint.velocity_controller.computeCommand(setpoint - joint.velocity, period),
        -joint.limits.max_effort, joint.limits.max_effort);
}

void PositionerHWSim::updatePositionerState(ros::Time time) {
    for (const auto& joint : this->joints_) {
        this->robot_state_.q = joint.second->position;
        this->robot_state_.dq = joint.second->velocity;
        
        this->robot_state_.q_d = joint.second->getDesiredPosition();
        this->robot_state_.dq_d = joint.second->getDesiredVelocity();
        this->robot_state_.ddq_d = joint.second->getDesiredAcceleration();
    }
}

bool PositionerHWSim::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) {
  return std::all_of(start_list.cbegin(), start_list.cend(), [this](const auto& controller) {
    return verifier_->isValidController(controller);
  });
}

void PositionerHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                           const std::list<hardware_interface::ControllerInfo>& stop_list) {
  forControlledJoint(stop_list, [](za_gazebo::Joint& joint, const za_gazebo::ControlMethod& /*method*/) {
    joint.control_method = boost::none;
    joint.stop_position = joint.position;
    joint.desired_position = joint.position;
    joint.desired_velocity = 0;
  });
  forControlledJoint(start_list, [](za_gazebo::Joint& joint, const za_gazebo::ControlMethod& method) {
    joint.control_method = method;
    // sets the desired joint position once for the effort interface
    joint.desired_position = joint.position;
    joint.desired_velocity = 0;
  });
}

void PositionerHWSim::forControlledJoint(
    const std::list<hardware_interface::ControllerInfo>& controllers,
    const std::function<void(za_gazebo::Joint& joint, const za_gazebo::ControlMethod&)>& f) {
  for (const auto& controller : controllers) {
    for (const auto& resource : controller.claimed_resources) {
      auto control_method = za_gazebo::ControllerVerifier::determineControlMethod(resource.hardware_interface);
      if (not control_method) {
        continue;
      }
      for (const auto& joint_name : resource.resources) {
        auto& joint = joints_.at(joint_name);
        f(*joint, control_method.value());
      }
    }
  }
}

} // namespace hydra_gazebo

PLUGINLIB_EXPORT_CLASS(hydra_gazebo::PositionerHWSim, gazebo_ros_control::RobotHWSim)