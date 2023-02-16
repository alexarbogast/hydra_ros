#pragma once

#include <gazebo_ros_control/robot_hw_sim.h>
#include <za_gazebo/za_hw_sim.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

//#define DEBUGGING_TOOLS

namespace hydra_gazebo
{
static const std::string HARDWARE_PARAM = "robot_hardware";
typedef std::vector<std::shared_ptr<gazebo_ros_control::RobotHWSim>> RobotHWList;

class HydraHWSim : public gazebo_ros_control::RobotHWSim
{
public:
    /**
    * Initialize the simulated robot hardware and parse all supported transmissions.
    *
    * @param[in] robot_namespace the name of the robot passed inside the `<robotNamespace>` tag from
    * the URDF
    * @param[in] model_nh root node handle of the node into which this plugin is loaded (usually
    * Gazebo)
    * @param[in] parent the underlying gazebo model type of the robot which was added
    * @param[in] urdf the parsed URDF which should be added
    * @param[in] transmissions a list of transmissions of the model which should be simulated
    * @return `true` if initialization succeeds, `false` otherwise
    */
    bool initSim(const std::string& robot_namespace,
                 ros::NodeHandle model_nh,
                 gazebo::physics::ModelPtr parent,
                 const urdf::Model* const urdf,
                 std::vector<transmission_interface::TransmissionInfo> transmissions) override;

    /**
    * @param[in] time   the current (simulated) ROS time
    * @param[in] period the time step at which the simulation is running
    */
    void readSim(ros::Time time, ros::Duration period) override;

    /**
    * Pass the data send from controllers via the hardware interfaces onto the simulation.
    *
    * This will e.g. write the joint commands (torques or forces) to the corresponding joint in
    * Gazebo in each timestep. These commands are usually send via an
    * [EffortJointInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1EffortJointInterface.html)
    *
    * @param[in] time   the current (simulated) ROS time
    * @param[in] period the time step at which the simulation is running
    */
    void writeSim(ros::Time time, ros::Duration period) override;

    /**
    * Switches the control mode of the robot arm
    * @param start_list list of controllers to start
    * @param stop_list list of controllers to stop
    */
    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    /**
    * Check (in non-realtime) if given controllers could be started and stopped from the current
    * state of the RobotHW with regard to necessary hardware interface switches and prepare the
    * switching. Start and stop list are disjoint. This handles the check and preparation, the actual
    * switch is commited in doSwitch().
    */
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& stop_list) override;

protected:
    bool loadRobotHW(const std::string& name, 
                     const urdf::Model* const urdf,
                     std::vector<transmission_interface::TransmissionInfo>& transmissions);

    void filterControllerList(const std::list<hardware_interface::ControllerInfo>& list,
                              std::list<hardware_interface::ControllerInfo>& filtered_list,
                              hardware_interface::RobotHWSharedPtr robot_hw,
                              const std::string& arm_id);

    bool robot_initialized_;
    pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim> robot_hw_loader_ 
        = {"gazebo_ros_control", "gazebo_ros_control::RobotHWSim"};

    gazebo::physics::ModelPtr robot_;
    std::array<double, 3> gravity_earth_;

    std::vector<std::string> arm_ids_;
    double tau_ext_lowpass_filter_;
    RobotHWList robot_hw_list_;

    ros::Publisher robot_initialized_pub_;
    ros::NodeHandle model_nh_;
};

// Debugging tools
#ifdef DEBUGGING_TOOLS
void printControllerInfo(const hardware_interface::ControllerInfo& ci) {
    std::cout << "Controller Name: " << ci.name << std::endl;
    std::cout << "Controller Type: " << ci.name << std::endl;

    for (const auto& resource : ci.claimed_resources) {
        std::cout << "\tResource:" << std::endl;
        std::cout << "\t\thardware interface: " << resource.hardware_interface
        << std::endl;

        std::cout << "\t\tResources: " << std::endl <<"\t\t\t";
        for (const auto& r : resource.resources) {
            std::cout << " " << r;
        }
        std::cout << std::endl << std::endl;
    }
}
#endif

} // namespace hydra_gazebo