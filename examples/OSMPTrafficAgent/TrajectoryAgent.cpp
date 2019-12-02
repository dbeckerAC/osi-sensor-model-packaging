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
#include <cmath>

int TrajectoryAgent::init()
{
    // initial values for the pose which is used as long as no
    // trajectory is passed via OSI::TrafficCommand
    pose.x = 0;
    pose.y = 0;
    pose.yaw = 0;
    trajSet = false;
    return 0;
}

int TrajectoryAgent::step(double time, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out)
{
    if(commandData.receive_new_command())
    {
        traj.CopyFrom(commandData.trajectory_command());
        trajSet = true;
    }
    return getTrajPoint(time, out);
    
}

int TrajectoryAgent::terminate()
{
    return 0;
}

int TrajectoryAgent::getTrajPoint(double time, osi3::TrafficUpdate &out)
{
    osi3::MovingObject *update = out.mutable_update();

    if(trajSet) 
    {
        //interpolate at current time step
        int i;
        double t1;
        osi3::TrajectoryPoint *tmp;
        for (i = 0; i < traj.trajectory_point_size(); i++)
        {
            tmp = traj.mutable_trajectory_point(i);
            t1 = tmp->time_stamp().seconds() 
               + tmp->time_stamp().nanos() / 1000000000.0;
            if(time > t1)
                break;
        }
        if (i+1 == traj.trajectory_point_size())
        {
            // when trajectory has ended, always set pose to last traj. point
            pose.x = tmp->base_moving().position().x();
            pose.y = tmp->base_moving().position().y();
            // take heading from traj. if not set, it is always zero (I assume)
            pose.yaw = tmp->base_moving().orientation().yaw();
        }
        else
        {
            // linear interpolation between both neighboring 
            // traj. points (w.r.t. time)
            osi3::TrajectoryPoint *tmp2; 
            tmp2 = traj.mutable_trajectory_point(i+1);
            double t2 = tmp2->time_stamp().seconds() 
                      + tmp2->time_stamp().nanos() / 1000000000.0;
            double x1 = tmp->base_moving().position().x();
            double x2 = tmp2->base_moving().position().x();
            double y1 = tmp->base_moving().position().y();
            double y2 = tmp2->base_moving().position().y();
            double mx = (x2-x1) / (t2-t1);
            double my = (y2-y1) / (t2-t1);
            pose.x = mx * (time-t1) + x1;
            pose.y = my * (time-t1) + y1;
            // orientation: either take from traj. or calculate slope
            // 1st: use linear interpolation for heading            
            double heading = std::atan2(y2-y1, x2-x1);
            pose.yaw = heading;
        }        
    }

    update->mutable_base()->mutable_position()->set_x(pose.x);
    update->mutable_base()->mutable_position()->set_y(pose.y);
    update->mutable_base()->mutable_orientation()->set_yaw(pose.yaw);

    return 0;
}
