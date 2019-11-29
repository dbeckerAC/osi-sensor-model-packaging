 /*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */ 
#include "TrajectoryAgent.h"

int TrajectoryAgent::init()
{
    trajSet = false;
    return 0;
}

int TrajectoryAgent::step(double time, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out)
{
    if(commandData.receive_new_command())
    {

    } else
    {
        if(trajSet)
        {
            return getTrajPoint(time, traj, out);
        } else 
        {
            // no trajectory yet. Decide what to do
            // either do nothing, or set init pos or something
        }
    }
    
    return 0;
}

int TrajectoryAgent::terminate()
{
    return 0;
}

int TrajectoryAgent::getTrajPoint(double time, osi3::TrajectoryCommand &trajectory, osi3::TrafficUpdate &out)
{
    osi3::MovingObject *update = out.mutable_update();
    update->mutable_base()->mutable_position()->set_x(11);
    update->mutable_base()->mutable_position()->set_y(12);
    update->mutable_base()->mutable_orientation()->set_yaw(0.5);
    return 0;
}