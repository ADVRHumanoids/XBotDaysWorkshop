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

#include <XBotDaysPlugin_rt_plugin.h>

/* Specify that the class XBotPlugin::XBotDaysPlugin is a XBot RT plugin with name "XBotDaysPlugin" */
REGISTER_XBOT_PLUGIN(XBotDaysPlugin, XBotPlugin::XBotDaysPlugin)

namespace XBotPlugin {

bool XBotDaysPlugin::init_control_plugin(std::string path_to_config_file,
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

    _logger = XBot::MatLogger::getLogger("/tmp/XBotDaysPlugin_log");

    /* Intialize/preallocate all variables */
    _enable_gcomp = true;
    _robot->getRobotState("home", _q_final);

    return true;


}

void XBotDaysPlugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /PholusPlugin_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _start_time = time;

    /* Save the robot starting config to a class member */
    _robot->getMotorPosition(_q0);
    _robot->getStiffness(_stiffness0);
    _robot->getDamping(_damping0);

    /* Set the robot in low impedance mode */
    _robot->setStiffness(_stiffness0 * 0.01);
    _robot->setDamping(_damping0 * 0.1);

    /* Write new impedance to eCAT buffer */
    _robot->move();
}

void XBotDaysPlugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /PholusPlugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    _robot->setStiffness(_stiffness0);
    _robot->setDamping(_damping0);
    _robot->move();

}


void XBotDaysPlugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    if(command.read(current_command)){
        if( current_command.str() == "gcomp ON" ){
            _enable_gcomp = true;
        }
        if( current_command.str() == "gcomp OFF" ){
            _enable_gcomp = false;
        }
    }

    /* Define a homing time */
    double homing_time = 5.0;

    /* Define the normalized time from the plugin start time to the homing time */
    double tau = (time - _start_time)/homing_time;

    /* Define a variable which goes smoothly from 0 to 1 over the homing time interval */
    double alpha = tau*tau*tau*(tau*(6*tau-15) + 10);

    /* Force alpha to be <= 1 */
    alpha = std::min(alpha, 1.0);

    /* Define the position reference */
    _qref = alpha * _q_final + (1 - alpha) * _q0;

    /* Compute gravity torque */
     _gcomp.setZero(_robot->getJointNum());

    if(_enable_gcomp) {
        /* Note: the internal model is always updated with the latest sensor readings! */
        _robot->model().computeGravityCompensation(_gcomp);
    }

    /* Send position and torque references */
    _robot->setPositionReference(_qref);
    _robot->setEffortReference(_gcomp);


    _robot->move();

}

bool XBotDaysPlugin::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}