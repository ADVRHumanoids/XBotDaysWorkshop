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

#ifndef XBotDays_Plugin_H_
#define XBotDays_Plugin_H_

#include <XCM/XBotControlPlugin.h>


namespace XBotPlugin {

/**
 * @brief XBotDaysPlugin XBot RT Plugin
 *
 **/
class XBotDaysPlugin : public XBot::XBotControlPlugin
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

    Eigen::VectorXd _q0, _stiffness0, _damping0;
    Eigen::VectorXd _qref, _gcomp, _q_final;

    bool _enable_gcomp;

    XBot::MatLogger::Ptr _logger;

};

}

#endif // XBotDaysPlugin_PLUGIN_H_