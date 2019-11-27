/*
 * Author: Daniel Becker
 * Based on: PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2019 Institute for Automotive Engineering, RWTH Aachen Univ.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPTrafficAgent.h"

/*
 * Debug Breaks
 *
 * If you define DEBUG_BREAKS the FMU will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the FMU is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#if defined(DEBUG_BREAKS) && !defined(NDEBUG)
#if defined(__has_builtin) && !defined(__ibmxl__)
#if __has_builtin(__builtin_debugtrap)
#define DEBUGBREAK() __builtin_debugtrap()
#elif __has_builtin(__debugbreak)
#define DEBUGBREAK() __debugbreak()
#endif
#endif
#if !defined(DEBUGBREAK)
#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#include <signal.h>
#if defined(SIGTRAP)
#define DEBUGBREAK() raise(SIGTRAP)
#else
#define DEBUGBREAK() raise(SIGABRT)
#endif
#endif
#endif
#else
#define DEBUGBREAK()
#endif

#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>
#include <cmath>

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPTrafficAgent::private_log_file;
#endif

/*
 * ProtocolBuffer Accessors
 */

void* decode_integer_to_pointer(fmi2Integer hi,fmi2Integer lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.base.lo=lo;
    myaddr.base.hi=hi;
    return reinterpret_cast<void*>(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return reinterpret_cast<void*>(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void encode_pointer_to_integer(const void* ptr,fmi2Integer& hi,fmi2Integer& lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.address=reinterpret_cast<unsigned long long>(ptr);
    hi=myaddr.base.hi;
    lo=myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    hi=0;
    lo=reinterpret_cast<int>(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}
/*
void COSMPTrafficAgent::reset_fmi_sensor_view_config_request()
{
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]=0;
}*/

bool COSMPTrafficAgent::get_fmi_sensor_view_in(osi3::SensorView& data)
{
    if (integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0) {
        void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
        normal_log("OSMP","Got %08X %08X, reading from %p ...",integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX],buffer);
        data.ParseFromArray(buffer,integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);
        return true;
    } else {
        return false;
    }
}

void COSMPTrafficAgent::set_fmi_traffic_update_out(const osi3::TrafficUpdate& data)
{
    data.SerializeToString(&currentOutputBuffer);
    encode_pointer_to_integer(currentOutputBuffer.data(),integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_SIZE_IDX]=(fmi2Integer)currentOutputBuffer.length();
    normal_log("OSMP","Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASELO_IDX],currentOutputBuffer.data());
    swap(currentOutputBuffer,lastOutputBuffer);
}

void COSMPTrafficAgent::reset_fmi_traffic_update_out()
{
    integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_TRAFFICUPDATE_OUT_BASELO_IDX]=0;
}

/*
 * Actual Core Content
 */

fmi2Status COSMPTrafficAgent::doInit()
{
    DEBUGBREAK();

    /* Booleans */
    for (int i = 0; i<FMI_BOOLEAN_VARS; i++)
        boolean_vars[i] = fmi2False;

    /* Integers */
    for (int i = 0; i<FMI_INTEGER_VARS; i++)
        integer_vars[i] = 0;

    /* Reals */
    for (int i = 0; i<FMI_REAL_VARS; i++)
        real_vars[i] = 0.0;

    /* Strings */
    for (int i = 0; i<FMI_STRING_VARS; i++)
        string_vars[i] = "";

    set_fmi_nominal_range(135.0);
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::doEnterInitializationMode()
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::doExitInitializationMode()
{
    DEBUGBREAK();

    /*osi3::SensorViewConfiguration config;
    if (!get_fmi_sensor_view_config(config))
        normal_log("OSI","Received no valid SensorViewConfiguration from Simulation Environment, assuming everything checks out.");
    else {
        normal_log("OSI","Received SensorViewConfiguration for Sensor Id %llu",config.sensor_id().value());
        normal_log("OSI","SVC Ground Truth FoV Horizontal %f, FoV Vertical %f, Range %f",config.field_of_view_horizontal(),config.field_of_view_vertical(),config.range());
        normal_log("OSI","SVC Mounting Position: (%f, %f, %f)",config.mounting_position().position().x(),config.mounting_position().position().y(),config.mounting_position().position().z());
        normal_log("OSI","SVC Mounting Orientation: (%f, %f, %f)",config.mounting_position().orientation().roll(),config.mounting_position().orientation().pitch(),config.mounting_position().orientation().yaw());
    }*/

    return fmi2OK;
}

void rotatePoint(double x, double y, double z,double yaw,double pitch,double roll,double &rx,double &ry,double &rz)
{
    double matrix[3][3];
    double cos_yaw = cos(yaw);
    double cos_pitch = cos(pitch);
    double cos_roll = cos(roll);
    double sin_yaw = sin(yaw);
    double sin_pitch = sin(pitch);
    double sin_roll = sin(roll);

    matrix[0][0] = cos_yaw*cos_pitch;  matrix[0][1]=cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll; matrix[0][2]=cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll;
    matrix[1][0] = sin_yaw*cos_pitch;  matrix[1][1]=sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll; matrix[1][2]=sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
    matrix[2][0] = -sin_pitch;         matrix[2][1]=cos_pitch*sin_roll;                            matrix[2][2]=cos_pitch*cos_roll;

    rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
    ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
    rz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

fmi2Status COSMPTrafficAgent::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    DEBUGBREAK();

    osi3::SensorView currentIn;
    osi3::TrafficUpdate currentOut;
    double time = currentCommunicationPoint+communicationStepSize;
    normal_log("OSI","Calculating Sensor at %f for %f (step size %f)",currentCommunicationPoint,time,communicationStepSize);
    if (get_fmi_sensor_view_in(currentIn)) {
    // TODO: do calculation here!
    /*
    * Todos: 
    *   * include osi::agentcommand as input
    *   * process trajectory in member variable
    *   * write interpolation function
    *   * ..
    *
    */
        /*double ego_x=0, ego_y=0, ego_z=0;
        osi3::Identifier ego_id = currentIn.global_ground_truth().host_vehicle_id();
        normal_log("OSI","Looking for EgoVehicle with ID: %llu",ego_id.value());
        for_each(currentIn.global_ground_truth().moving_object().begin(),currentIn.global_ground_truth().moving_object().end(),
            [this, ego_id, &ego_x, &ego_y, &ego_z](const osi3::MovingObject& obj) {
                normal_log("OSI","MovingObject with ID %llu is EgoVehicle: %d",obj.id().value(), obj.id().value() == ego_id.value());
                if (obj.id().value() == ego_id.value()) {
                    normal_log("OSI","Found EgoVehicle with ID: %llu",obj.id().value());
                    ego_x = obj.base().position().x();
                    ego_y = obj.base().position().y();
                    ego_z = obj.base().position().z();
                }
            });
        normal_log("OSI","Current Ego Position: %f,%f,%f", ego_x, ego_y, ego_z);*/

        /* Clear Output */
        currentOut.Clear();
        currentOut.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
        /* Adjust Timestamps and Ids */
        currentOut.mutable_timestamp()->set_seconds((long long int)floor(time));
        currentOut.mutable_timestamp()->set_nanos((int)((time - floor(time))*1000000000.0));
        
        /*int i=0;
        double actual_range = fmi_nominal_range()*1.1;
        for_each(currentIn.global_ground_truth().moving_object().begin(),currentIn.global_ground_truth().moving_object().end(),
            [this,&i,&currentIn,&currentOut,ego_id,ego_x,ego_y,ego_z,actual_range](const osi3::MovingObject& veh) {
                if (veh.id().value() != ego_id.value()) {
                    // NOTE: We currently do not take sensor mounting position into account,
                    // i.e. sensor-relative coordinates are relative to center of bounding box
                    // of ego vehicle currently.
                    double trans_x = veh.base().position().x()-ego_x;
                    double trans_y = veh.base().position().y()-ego_y;
                    double trans_z = veh.base().position().z()-ego_z;
                    double rel_x,rel_y,rel_z;
                    rotatePoint(trans_x,trans_y,trans_z,veh.base().orientation().yaw(),veh.base().orientation().pitch(),veh.base().orientation().roll(),rel_x,rel_y,rel_z);
                    double distance = sqrt(rel_x*rel_x + rel_y*rel_y + rel_z*rel_z);
                    if ((distance <= actual_range) && (rel_x/distance > 0.866025)) {
                        osi3::DetectedMovingObject *obj = currentOut.mutable_moving_object()->Add();
                        obj->mutable_header()->add_ground_truth_id()->CopyFrom(veh.id());
                        obj->mutable_header()->mutable_tracking_id()->set_value(i);
                        obj->mutable_header()->set_existence_probability(cos((2.0*distance-actual_range)/actual_range));
                        obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
                        obj->mutable_header()->add_sensor_id()->CopyFrom(currentIn.sensor_id());
                        obj->mutable_base()->mutable_position()->set_x(rel_x);
                        obj->mutable_base()->mutable_position()->set_y(rel_y);
                        obj->mutable_base()->mutable_position()->set_z(rel_z);
                        obj->mutable_base()->mutable_dimension()->set_length(veh.base().dimension().length());
                        obj->mutable_base()->mutable_dimension()->set_width(veh.base().dimension().width());
                        obj->mutable_base()->mutable_dimension()->set_height(veh.base().dimension().height());
                        
                        osi3::DetectedMovingObject::CandidateMovingObject* candidate = obj->add_candidate();
                        candidate->set_type(veh.type());
                        candidate->mutable_vehicle_classification()->CopyFrom(veh.vehicle_classification());
                        candidate->set_probability(1);
                        
                        normal_log("OSI","Output Vehicle %d[%llu] Probability %f Relative Position: %f,%f,%f (%f,%f,%f)",i,veh.id().value(),obj->header().existence_probability(),rel_x,rel_y,rel_z,obj->base().position().x(),obj->base().position().y(),obj->base().position().z());
                        i++;
                    } else {
                        normal_log("OSI","Ignoring Vehicle %d[%llu] Outside Sensor Scope Relative Position: %f,%f,%f (%f,%f,%f)",i,veh.id().value(),veh.base().position().x()-ego_x,veh.base().position().y()-ego_y,veh.base().position().z()-ego_z,veh.base().position().x(),veh.base().position().y(),veh.base().position().z());
                    }
                }
                else
                {
                    normal_log("OSI","Ignoring EGO Vehicle %d[%llu] Relative Position: %f,%f,%f (%f,%f,%f)",i,veh.id().value(),veh.base().position().x()-ego_x,veh.base().position().y()-ego_y,veh.base().position().z()-ego_z,veh.base().position().x(),veh.base().position().y(),veh.base().position().z());
                }
            });
        normal_log("OSI","Mapped %d vehicles to output", i);*/
        /* Serialize */
        set_fmi_traffic_update_out(currentOut);
        set_fmi_valid(true);
        //set_fmi_count(currentOut.moving_object_size());
        set_fmi_count(1); // correct?
    } else {
        /* We have no valid input, so no valid output */
        normal_log("OSI","No valid input, therefore providing no valid output.");
        reset_fmi_traffic_update_out();
        set_fmi_valid(false);
        set_fmi_count(0);
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::doTerm()
{
    DEBUGBREAK();

    return fmi2OK;
}

void COSMPTrafficAgent::doFree()
{
    DEBUGBREAK();
}

/*
 * Generic C++ Wrapper Code
 */

COSMPTrafficAgent::COSMPTrafficAgent(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn)
    : instanceName(theinstanceName),
    fmuType(thefmuType),
    fmuGUID(thefmuGUID),
    fmuResourceLocation(thefmuResourceLocation),
    functions(*thefunctions),
    visible(!!thevisible),
    loggingOn(!!theloggingOn),
    simulation_started(false)
{
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("OSI");
}

COSMPTrafficAgent::~COSMPTrafficAgent()
{

}


fmi2Status COSMPTrafficAgent::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[])
{
    fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn ? true : false;
    if (categories && (nCategories > 0)) {
        loggingCategories.clear();
        for (size_t i=0;i<nCategories;i++) {
            if (categories[i] == "FMI")
                loggingCategories.insert("FMI");
            else if (categories[i] == "OSMP")
                loggingCategories.insert("OSMP");
            else if (categories[i] == "OSI")
                loggingCategories.insert("OSI");
        }
    } else {
        loggingCategories.clear();
        loggingCategories.insert("FMI");
        loggingCategories.insert("OSMP");
        loggingCategories.insert("OSI");
    }
    return fmi2OK;
}

fmi2Component COSMPTrafficAgent::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
    COSMPTrafficAgent* myc = new COSMPTrafficAgent(instanceName,fmuType,fmuGUID,fmuResourceLocation,functions,visible,loggingOn);

    if (myc == NULL) {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn);
        return NULL;
    }

    if (myc->doInit() != fmi2OK) {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn);
        delete myc;
        return NULL;
    }
    else {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn, myc);
        return (fmi2Component)myc;
    }
}

fmi2Status COSMPTrafficAgent::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status COSMPTrafficAgent::EnterInitializationMode()
{
    fmi_verbose_log("fmi2EnterInitializationMode()");
    return doEnterInitializationMode();
}

fmi2Status COSMPTrafficAgent::ExitInitializationMode()
{
    fmi_verbose_log("fmi2ExitInitializationMode()");
    simulation_started = true;
    return doExitInitializationMode();
}

fmi2Status COSMPTrafficAgent::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status COSMPTrafficAgent::Terminate()
{
    fmi_verbose_log("fmi2Terminate()");
    return doTerm();
}

fmi2Status COSMPTrafficAgent::Reset()
{
    fmi_verbose_log("fmi2Reset()");

    doFree();
    simulation_started = false;
    return doInit();
}

void COSMPTrafficAgent::FreeInstance()
{
    fmi_verbose_log("fmi2FreeInstance()");
    doFree();
}

fmi2Status COSMPTrafficAgent::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    fmi_verbose_log("fmi2GetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            value[i] = real_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    fmi_verbose_log("fmi2GetInteger(...)");
    bool need_refresh = !simulation_started;
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS) {
            /*if (need_refresh && (vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX)) {
                refresh_fmi_sensor_view_config_request();
                need_refresh = false;
            }*/
            value[i] = integer_vars[vr[i]];
        } else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    fmi_verbose_log("fmi2GetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            value[i] = boolean_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    fmi_verbose_log("fmi2GetString(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            value[i] = string_vars[vr[i]].c_str();
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    fmi_verbose_log("fmi2SetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            real_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    fmi_verbose_log("fmi2SetInteger(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            integer_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    fmi_verbose_log("fmi2SetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            boolean_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPTrafficAgent::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    fmi_verbose_log("fmi2SetString(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            string_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

extern "C" {

    FMI2_Export const char* fmi2GetTypesPlatform()
    {
        return fmi2TypesPlatform;
    }

    FMI2_Export const char* fmi2GetVersion()
    {
        return fmi2Version;
    }

    FMI2_Export fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetDebugLogging(loggingOn, nCategories, categories);
    }

    /*
    * Functions for Co-Simulation
    */
    FMI2_Export fmi2Component fmi2Instantiate(fmi2String instanceName,
        fmi2Type fmuType,
        fmi2String fmuGUID,
        fmi2String fmuResourceLocation,
        const fmi2CallbackFunctions* functions,
        fmi2Boolean visible,
        fmi2Boolean loggingOn)
    {
        return COSMPTrafficAgent::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
    }

    FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
        fmi2Boolean toleranceDefined,
        fmi2Real tolerance,
        fmi2Real startTime,
        fmi2Boolean stopTimeDefined,
        fmi2Real stopTime)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    }

    FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->EnterInitializationMode();
    }

    FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->ExitInitializationMode();
    }

    FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
        fmi2Real currentCommunicationPoint,
        fmi2Real communicationStepSize,
        fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    }

    FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->Terminate();
    }

    FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->Reset();
    }

    FMI2_Export void fmi2FreeInstance(fmi2Component c)
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        myc->FreeInstance();
        delete myc;
    }

    /*
     * Data Exchange Functions
     */
    FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->GetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->GetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->GetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->GetString(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
    {
        COSMPTrafficAgent* myc = (COSMPTrafficAgent*)c;
        return myc->SetString(vr, nvr, value);
    }

    /*
     * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
     */
    FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t *size)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SerializeFMUstate (fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2DeSerializeFMUstate (fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
        const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
        const fmi2ValueReference vKnown_ref[] , size_t nKnown,
        const fmi2Real dvKnown[],
        fmi2Real dvUnknown[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SetRealInputDerivatives(fmi2Component c,
        const  fmi2ValueReference vr[],
        size_t nvr,
        const  fmi2Integer order[],
        const  fmi2Real value[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c,
        const   fmi2ValueReference vr[],
        size_t  nvr,
        const   fmi2Integer order[],
        fmi2Real value[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c)
    {
        return fmi2OK;
    }

    FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
    {
        return fmi2Discard;
    }

}
