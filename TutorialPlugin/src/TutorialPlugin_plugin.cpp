/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <TutorialPlugin_plugin.h>

/* Specify that the class XBotPlugin::TutorialPlugin is a XBot RT plugin with name "TutorialPlugin" */
REGISTER_XBOT_PLUGIN(TutorialPlugin, XBotPlugin::TutorialPlugin)

namespace XBotPlugin {

bool TutorialPlugin::init_control_plugin(std::string path_to_config_file,
                                                    XBot::SharedMemory::Ptr shared_memory,
                                                    XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */

    _logger = XBot::MatLogger::getLogger("/tmp/TutorialPlugin_log");
    
    _enable_gcomp = false;
    _say_hello = false;
    
    robot->getRobotState("home", _q_final);
    robot->getRobotState("wave1", _q_wave1);
    robot->getRobotState("wave2", _q_wave2);

    return true;


}

void TutorialPlugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /TutorialPlugin_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _start_time = time;

    /* Save the robot starting config to a class member */
    _robot->getMotorPosition(_q0);
    _robot->getStiffness(_stiffness0);
    _robot->getDamping(_damping0);
    
    /* Set the robot in low impedence mode */
    _robot->setStiffness(_stiffness0 * 1);
    _robot->setStiffness(_damping0 * 1);
    
    /* write to robot */
    _robot->move();
}

void TutorialPlugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /TutorialPlugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    _robot->setStiffness(_stiffness0);
    _robot->setDamping(_damping0);
    _robot->move();
}


void TutorialPlugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /TutorialPlugin_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    Eigen::VectorXd current_motor_positions;
    _robot->getMotorPosition(current_motor_positions);
    _robot->setPositionReference(current_motor_positions);
    _robot->move();
      
//       std::cout << "current: " << current_motor_positions << std::endl;
//       std::cout << "_q0:     " << _q0 << std::endl;
//     
//     if(command.read(current_command)){
// 
//         if(current_command.str() == "gcomp ON"){
//             _enable_gcomp = true;
//         }
// 
//         if(current_command.str() == "gcomp OFF"){
//             _enable_gcomp = false;
//         }
//         
//         if(current_command.str() == "say hello"){
// 	  _say_hello = true;
// 	  _start_time = time;
// 	}
// 
//     }
//     
//     if (!_say_hello){
//       // Define action time
//       double action_time = 5.0;
//       
//       // Define relative action status
//       double rel_action_status = (time - _start_time) / action_time;
//       double smooth_action_status = rel_action_status * rel_action_status * rel_action_status * (rel_action_status * (6 * rel_action_status-15) + 10);
//       smooth_action_status = std::min(1.0, smooth_action_status);
//       
//       // Define current pose
//       _q_tmp = _q0;
// //       _q_tmp = _q0 + smooth_action_status * (_q_final - _q0);
//       
//       if (_enable_gcomp)
// 	_robot->model().computeGravityCompensation(_gcomp);
//       
//       else
// 	_gcomp.setZero(_robot->getJointNum());
//       
//       
//       // send to robot
//       _robot->setPositionReference(_q_tmp);
//       _robot->setEffortReference(_gcomp);
//       _robot->move();
//     }
//     else
//     {
//       // Define action time
//       double action_time = 2.0;
//       
//       // Define relative action status
//       double rel_action_status = (time - _start_time) / action_time;
//       rel_action_status = std::min(7.0, rel_action_status);
//       
//       if (rel_action_status <= 1) {
// 	// Define current pose
// 	_q_tmp = _q_final + rel_action_status * (_q_wave1 - _q_final);
//       }
//       else if (rel_action_status <= 2) {
// 	// Define current pose
// 	_q_tmp = _q_wave1 + (rel_action_status - 1) * (_q_wave2 - _q_wave1);
//       }
//       else if (rel_action_status <= 3) {
// 	// Define current pose
// 	_q_tmp = _q_wave2 + (rel_action_status - 2) * (_q_wave1 - _q_wave2);
//       }
//       else if (rel_action_status <= 4) {
// 	// Define current pose
// 	_q_tmp = _q_wave1 + (rel_action_status - 3) * (_q_wave2 - _q_wave1);
//       }
//       else if (rel_action_status <= 5) {
// 	// Define current pose
// 	_q_tmp = _q_wave2 + (rel_action_status - 4) * (_q_wave1 - _q_wave2);
//       }
//       else if (rel_action_status <= 6) {
// 	// Define current pose
// 	_q_tmp = _q_wave1 + (rel_action_status - 5) * (_q_wave2 - _q_wave1);
//       }
//       else if (rel_action_status <= 7) {
// 	// Define current pose
// 	_q_tmp = _q_wave2 + (rel_action_status - 6) * (_q_final - _q_wave2);
//       }
//       
//       if (rel_action_status > 7 - 1e-3)
// 	_say_hello = false;
//       
//       _robot->model().computeGravityCompensation(_gcomp);
//       
//       // send to robot
//       _robot->setPositionReference(_q_tmp);
//       _robot->setEffortReference(_gcomp);
//       _robot->move();
//     }
//     

}

bool TutorialPlugin::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
