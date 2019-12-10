/*
 * Author: Daniel Becker
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */ 
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "osi_sensorview.pb.h"

struct Pose {
    double x;
    double y;
    double yaw;
};

class TrajectoryAgent {
public:
    TrajectoryAgent() {}
    ~TrajectoryAgent() {}

    int init();
    int step(double time, osi3::SensorView &sensorViewData, osi3::TrafficCommand &commandData, osi3::TrafficUpdate &out);
    int terminate();

private:
    bool trajSet;
    osi3::TrajectoryAction traj;
    Pose pose;

    int getTrajPoint(double time, osi3::TrafficUpdate &out);
};