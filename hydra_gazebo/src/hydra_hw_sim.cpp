#include <hydra_gazebo/hydra_hw_sim.h>
#include <hydra_gazebo/model_kdl.h>
#include <hydra_hw/positioner_state_interface.h>
#include <std_msgs/Bool.h>

namespace hydra_gazebo
{
bool HydraHWSim::initSim(const std::string& robot_namespace,
                         ros::NodeHandle model_nh,
                         gazebo::physics::ModelPtr parent,
                         const urdf::Model* const urdf,
                         std::vector<transmission_interface::TransmissionInfo> transmissions) {
    this->robot_ = parent;
    this->model_nh_ = model_nh;

    if (!model_nh_.getParam(HARDWARE_PARAM, robot_hw_ids_)) {
        ROS_ERROR_STREAM("Could not find " << HARDWARE_PARAM << 
                         "' parameter (namespace: " << model_nh_.getNamespace() << ").");
        return false;
    }

    for (const auto& robot : robot_hw_ids_) {
        std::string arm_id;
        ros::NodeHandle nh(robot);
        if (!nh.getParam("arm_id", arm_id)) {
            return false;            
        }
        this->arm_ids_.push_back(arm_id);

        if (!loadRobotHW(robot, urdf, transmissions)) {
            return false;
        }
    }

    for (const auto& transmission : transmissions) {
        if (transmission.type_ == "hydra_hw/HydraModelInterface") {
                ROS_INFO_STREAM_NAMED("hydra_hw_sim",
                    "Found transmission interface '" << transmission.type_ << "'");
                try {
                    initHydraModelHandle(model_nh, *urdf, transmission);
                    continue;
                } catch (const std::invalid_argument& e) {
                        ROS_ERROR_STREAM_NAMED("hydra_hw_sim", e.what());
                        return false;
                }
        }
    }
    registerInterface(&this->hmi_);
    return true;
}

void HydraHWSim::initHydraModelHandle(
    const ros::NodeHandle& nh,
    const urdf::Model& urdf,
    const transmission_interface::TransmissionInfo& transmission) {

    std::map<std::string, std::shared_ptr<za_hw::ZaStateHandle>> za_state_map;
    std::shared_ptr<hydra_hw::PositionerStateHandle> pos_state_handle;
    try {
        std::vector<std::string> tips, filtered_arm_ids;
        std::string positioner_frame = "positioner_frame";
        for (const auto& hw_id : robot_hw_ids_) {
            ros::NodeHandle node_handle(nh, hw_id);

            // find end_effector links for model. at this point we can assume
            // all registered interfaces of type ZaHWSim have a defined end_effector 
            std::string type;
            if (node_handle.getParam("type", type)) {
                if (type == "za_gazebo/ZaHWSim") {
                    std::string eef_link, arm_id;
                    node_handle.getParam("end_effector", eef_link);
                    node_handle.getParam("arm_id", arm_id);
                    tips.push_back(eef_link);
                    filtered_arm_ids.push_back(arm_id);
                }
                else if (type == "hydra_gazebo/PositionerHWSim") {
                    if (not node_handle.getParam("frame", positioner_frame)) {
                        ROS_ERROR_STREAM("Could not find 'frame" 
                         "' parameter (namespace: " << node_handle.getNamespace() << ").");
                    }
                }
            }   
        }

        for (auto& hw : this->robot_hw_map_) {
            auto* state_interface_ = hw.second->get<za_hw::ZaStateInterface>();
            if (state_interface_ != nullptr) {
                auto za_state_ptr = std::make_shared<za_hw::ZaStateHandle>(
                    state_interface_->getHandle(hw.first + "_robot"));
                za_state_map.emplace(std::make_pair(hw.first, std::move(za_state_ptr)));
                continue;
            }
            
            auto* pos_state_interface_ = hw.second->get<hydra_hw::PositionerStateInterface>();
            if (pos_state_interface_ != nullptr) {
                pos_state_handle = std::make_shared<hydra_hw::PositionerStateHandle>(
                    pos_state_interface_->getHandle(hw.first + "_robot"));
                continue;
            }
        }

        this->model_ =
            std::make_unique<hydra_gazebo::ModelKDL>(
                urdf, filtered_arm_ids, tips, positioner_frame);
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("Cannot create hydra_hw/HydraModelInterface");
    }

    this->hmi_.registerHandle(
        hydra_hw::HydraModelHandle("hydra_model", *this->model_, za_state_map, pos_state_handle));
}

void HydraHWSim::readSim(ros::Time time, ros::Duration period) {
    for (const auto& robot_hw : robot_hw_map_) {
        robot_hw.second->readSim(time, period);
    }
}

void HydraHWSim::writeSim(ros::Time time, ros::Duration period) {
    for (const auto& robot_hw : robot_hw_map_) {
        robot_hw.second->writeSim(time, period);
    }
}

bool HydraHWSim::loadRobotHW(const std::string& name,
                             const urdf::Model* const urdf,
                             std::vector<transmission_interface::TransmissionInfo>& transmissions) {
    ROS_INFO_STREAM("Loading hw_interface " << name);

    ros::NodeHandle c_nh;
    try {
        c_nh = ros::NodeHandle(model_nh_, name);
    }
    catch(const std::exception& e) {
        ROS_ERROR_STREAM("Could not construct node handle for hw name " << name);
        return false;
    }

    // parse hw_sim type and load hw plugin
    std::shared_ptr<gazebo_ros_control::RobotHWSim> robot_hw;
    std::string type;
    if (not c_nh.getParam("type", type)) {
        ROS_ERROR_STREAM("Could not load robot HW: " << name << 
                         " because the type is not specified");
        return false;
    }

    try {
        for (const auto& cur_type : robot_hw_loader_.getDeclaredClasses()) {
            if (type == cur_type) {
                robot_hw = robot_hw_loader_.createUniqueInstance(type);
            }
        }
    }
    catch(const std::exception& e) {
        ROS_ERROR("Could not load class %s: %s", type.c_str(), e.what());
        return false;
    }

    // the hardware does not exist, exit
    if (!robot_hw) {
        ROS_ERROR_STREAM("Could not load robot HW because the type" <<
                         type << "does not exist");
        return false;
    }

    // initialize the hw with belonging transmissions
    std::string arm_id;
    if (not c_nh.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("Failed to initialize simulation hardware");
        return false;
    }
    
    std::vector<transmission_interface::TransmissionInfo> filtered_trans;
    std::copy_if(transmissions.begin(), transmissions.end(), std::back_inserter(filtered_trans),
        [arm_id](transmission_interface::TransmissionInfo& ti) {
            return ti.name_.find(arm_id) != std::string::npos; 
        });

    if (not robot_hw->initSim(arm_id, c_nh, robot_, urdf, filtered_trans)) {
        ROS_ERROR("Failed to initialize simulation hardware");
        return false;
    }

    robot_hw_map_.emplace(std::make_pair(arm_id, robot_hw));
    this->registerInterfaceManager(robot_hw.get());

    ROS_INFO("Successfully loaded robot HW '%s'", name.c_str());
    return true;
}

bool HydraHWSim::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list) {
    if (robot_hw_map_.size() != arm_ids_.size()) {
        ROS_ERROR_STREAM_NAMED("hydra_hw_sim", "Failed to prepare switch: " <<
            "the number of robot_hw does not match the number of arm ids.");
        return false;
    }

    auto robot_hw = robot_hw_map_.begin();
    auto arm_id = this->arm_ids_.begin();

    for (; robot_hw != robot_hw_map_.end(); robot_hw++, arm_id++) {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw->second, *arm_id);
        filterControllerList(stop_list, filtered_stop_list, robot_hw->second, *arm_id);

        #ifdef DEBUGGING_TOOLS
            std::cout << "Preparing Switch for armid: " << *arm_id << std::endl;
            std::cout << "Original start list" << std::endl;
            for (const auto& cont : start_list) {
                std::cout << "\t- " << cont.name << std::endl; 
            }

            std::cout << "Filtered start list" << std::endl;
            for (const auto& cont : filtered_start_list) {
                std::cout << "\t- " << cont.name << std::endl; 
            }
            std::cout << std::endl;
        #endif

        if (not (robot_hw->second)->prepareSwitch(filtered_start_list, filtered_stop_list)) {
            return false;
        }
    }
    return true;
}

void HydraHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                           const std::list<hardware_interface::ControllerInfo>& stop_list) {
    if (robot_hw_map_.size() != arm_ids_.size()) {
        ROS_ERROR_STREAM_NAMED("hydra_hw_sim", "Failed to prepare switch: " <<
            "the number of robot_hw does not match the number of arm ids.");
    }

    auto robot_hw = robot_hw_map_.begin();
    auto arm_id = this->arm_ids_.begin();

    for (; robot_hw != robot_hw_map_.end(); robot_hw++, arm_id++) {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw->second, *arm_id);
        filterControllerList(stop_list, filtered_stop_list, robot_hw->second, *arm_id);

        robot_hw->second->doSwitch(filtered_start_list, filtered_stop_list);
    }
}

void HydraHWSim::filterControllerList(
        const std::list<hardware_interface::ControllerInfo>& list,
        std::list<hardware_interface::ControllerInfo>& filtered_list,
        hardware_interface::RobotHWSharedPtr robot_hw,
        const std::string& arm_id) {

    filtered_list.clear();
    for (const auto& controller : list) {
        hardware_interface::ControllerInfo filtered_controller;
        filtered_controller.name = controller.name;
        filtered_controller.type = controller.type;

        if (controller.claimed_resources.empty()) {
            filtered_list.push_back(filtered_controller);
            continue;
        }

        // ZaStateControllers have no resources. Prevent filtering if the
        // controller is a ZaStateController and the arm_id is found in the 
        // controller name
        bool is_custom_state_controller = controller.type == "za_control/ZaStateController" or
                                          controller.type == "hydra_control/PositionerStateController";
        if (is_custom_state_controller) {
            if (controller.name.find(arm_id) != std::string::npos) {
                filtered_controller.claimed_resources = controller.claimed_resources;
                filtered_list.push_back(filtered_controller);
            }
            continue;
        }

        for (const auto& claimed_resource : controller.claimed_resources) {
            hardware_interface::InterfaceResources filtered_iface_resources;
            filtered_iface_resources.hardware_interface = claimed_resource.hardware_interface;
            std::vector<std::string> r_hw_ifaces = robot_hw->getNames();

            auto const name_match = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(),
                filtered_iface_resources.hardware_interface);
            if (name_match == r_hw_ifaces.end()) {
                continue;
            }

            std::vector<std::string> r_hw_iface_resources = 
                robot_hw->getInterfaceResources(filtered_iface_resources.hardware_interface);
            std::set<std::string> filtered_resources;
            for (const auto& resource : claimed_resource.resources) {
                std::vector<std::string>::iterator res_name = 
                    std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), resource);

                if (res_name != r_hw_iface_resources.end()) {
                    filtered_resources.insert(resource);
                }
            }

            if (!filtered_resources.empty()) {
                filtered_iface_resources.resources = filtered_resources;
                filtered_controller.claimed_resources.push_back(filtered_iface_resources);
            }
        }
        if (!filtered_controller.claimed_resources.empty()) {
            filtered_list.push_back(filtered_controller);
        }   
    }
}

} // namespace hydra_gazebo

PLUGINLIB_EXPORT_CLASS(hydra_gazebo::HydraHWSim, gazebo_ros_control::RobotHWSim)