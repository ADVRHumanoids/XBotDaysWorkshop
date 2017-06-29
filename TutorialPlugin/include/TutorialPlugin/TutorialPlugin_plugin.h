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

#ifndef TutorialPlugin_PLUGIN_H_
#define TutorialPlugin_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>


namespace XBotPlugin {

/**
 * @brief TutorialPlugin XBot RT Plugin
 *
 **/
class TutorialPlugin : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(std::string path_to_config_file,
                                     XBot::SharedMemory::Ptr shared_memory,
                                     XBot::RobotInterface::Ptr robot);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;

    double _start_time;

    Eigen::VectorXd _q0;
    Eigen::VectorXd _q_final;
    Eigen::VectorXd _q_wave1;
    Eigen::VectorXd _q_wave2;
    Eigen::VectorXd _q_tmp;
    Eigen::VectorXd _stiffness0;
    Eigen::VectorXd _damping0;
    Eigen::VectorXd _gcomp;

    XBot::MatLogger::Ptr _logger;
    
    bool _enable_gcomp;
    bool _say_hello;

};

}

#endif // TutorialPlugin_PLUGIN_H_
