// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RosFlightDroneController_hpp
#define msr_airlib_RosFlightDroneController_hpp

#include "controllers/DroneControllerBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "AirSimRosFlightBoard.hpp"
#include "vehicles/MultiRotorParams.hpp"

namespace msr { namespace airlib {

class RosFlightDroneController : public DroneControllerBase {

public:
    RosFlightDroneController(const MultiRotorParams* vehicle_params, const SensorCollection* sensors,
        const Environment* environment, const Kinematics::State* kinematics)
        : environment_(environment), kinematics_(kinematics), sensors_(sensors), vehicle_params_(vehicle_params)
    {
    }

public:
    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override
    {
        board_->system_reset(false);
    }

    virtual void update(real_T dt) override
    {
        //board has reference to sensor collection and doesn't need upates for sensors
    }

    virtual void start() override
    {
        board_.reset(new AirSimRosFlightBoard(&vehicle_params_->getParams().enabled_sensors, sensors_));
    }
    virtual void stop() override
    {
        board_.reset();
    }

    virtual size_t getVertexCount() override
    {
        return vehicle_params_->getParams().rotor_count;
    }

    virtual real_T getVertexControlSignal(unsigned int rotor_index) override
    {
        return board_->getMotorControlSignal(rotor_index);
    }

    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        //TODO: implement this
    }

    virtual bool isOffboardMode() override
    {
        //TODO: support offboard mode
        return false;
    }

    virtual bool isSimulationMode() override
    {
        return true;
    }

    virtual void setOffboardMode(bool is_set) override
    {
        //TODO: implement this
    }

    virtual void setSimulationMode(bool is_set) override
    {
        if (!is_set)
            throw VehicleCommandNotImplementedException("setting non-simulation mode is not supported yet");
    }
    //*** End: VehicleControllerBase implementation ***//

//*** Start: DroneControllerBase implementation ***//
public:
    Vector3r getPosition() override
    {
        return kinematics_->pose.position;
    }

    Vector3r getVelocity() override
    {
        return kinematics_->twist.linear;
    }

    Quaternionr getOrientation() override
    {
        return kinematics_->pose.orientation;
    }

    RCData getRCData() override
    {
        //TODO: implement this
    }

    void setRCData(const RCData& rcData)
    {
        //TODO: implement this
    }
    
    double timestampNow() override
    {
        return Utils::getTimeSinceEpoch();
    }

    bool armDisarm(bool arm, CancelableBase& cancelable_action) override
    {
        //TODO: implement this
    }

    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override
    {
        //TODO: implement this
    }

    bool land(CancelableBase& cancelable_action) override
    {
        //TODO: implement this
    }

    bool goHome(CancelableBase& cancelable_action) override
    {
        //TODO: implement this
    }

    bool hover(CancelableBase& cancelable_action) override
    {
        //TODO: implement this
    }

    GeoPoint getHomePoint() override
    {
        environment_->getInitialState().geo_point;
    }

    GeoPoint getGpsLocation() override
    {
        environment_->getState().geo_point;
    }

    virtual void reportTelemetry(float renderTime) override
    {
        //TODO: implement this
    }

    float getCommandPeriod() override
    {
        return 1.0f/50; //50hz
    }

    float getTakeoffZ() override
    {
        // pick a number, 3 meters is probably safe 
        // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
        return -3.0f;  
    }

    float getDistanceAccuracy() override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }

protected: 
    void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        //TODO: implement this
    }

    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    const VehicleParams& getVehicleParams() override
    {
        //used for safety algos. For now just use defaults
        static const VehicleParams vehicle_params_;
        return vehicle_params_;
    }
    //*** End: DroneControllerBase implementation ***//

private:
    const MultiRotorParams* vehicle_params_;
    const Environment* environment_;
    const SensorCollection* sensors_;
    const Kinematics::State* kinematics_;
    unique_ptr<AirSimRosFlightBoard> board_;
};

}} //namespace
#endif 