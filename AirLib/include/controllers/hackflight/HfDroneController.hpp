// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ros_HfDroneController_hpp
#define air_ros_HfDroneController_hpp

#include "controllers/DroneControllerBase.hpp"
#include "sensors/SensorCollection.hpp"

namespace msr { namespace airlib {


class HfDroneController : public DroneControllerBase
{
//public:
//    //*** Start: VehicleControllerBase implementation ***//
//    virtual void reset() override;
//    virtual void update(real_T dt) override;
//    virtual void start() override;
//    virtual void stop() override;
//    virtual size_t getVertexCount() override;
//    virtual real_T getVertexControlSignal(unsigned int rotor_index) override;
//    virtual void getStatusMessages(std::vector<std::string>& messages) override;
//
//    virtual bool isOffboardMode() override;
//    virtual bool isSimulationMode() override;
//    virtual void setOffboardMode(bool is_set) override;
//    virtual void setSimulationMode(bool is_set) override;
//    virtual void setUserInputs(const vector<float>& inputs) override;
//    //*** End: VehicleControllerBase implementation ***//
//
//
//    //*** Start: DroneControllerBase implementation ***//
//public:
//    Vector3r getPosition() override;
//    Vector3r getVelocity() override;
//    Quaternionr getOrientation() override;
//    RCData getRCData() override;
//    double timestampNow() override;
//
//    bool armDisarm(bool arm, CancelableBase& cancelable_action) override;
//    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override;
//    bool land(CancelableBase& cancelable_action) override;
//    bool goHome(CancelableBase& cancelable_action) override; 
//    bool hover(CancelableBase& cancelable_action) override;
//    GeoPoint getHomePoint() override;
//    GeoPoint getGpsLocation() override;
//    virtual void reportTelemetry(float renderTime) override;
//
//    float getCommandPeriod() override;
//    float getTakeoffZ() override;
//    float getDistanceAccuracy() override;
//protected: 
//    void commandRollPitchZ(float pitch, float roll, float z, float yaw) override;
//    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override;
//    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override;
//    void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override;
//    void commandVirtualRC(const RCData& rc_data) override;
//    void commandEnableVirtualRC(bool enable) override;
//    const VehicleParams& getVehicleParams() override;
//    //*** End: DroneControllerBase implementation ***//
};

}} //namespace
#endif
