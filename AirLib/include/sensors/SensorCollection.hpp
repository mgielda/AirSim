// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_SensorBase_hpp
#define msr_air_copter_sim_SensorBase_hpp

#include <unordered_map>
#include "SensorBase.hpp"
#include "common\UpdatableContainer.hpp"
#include "common/Common.hpp"

#include "barometer/BarometerBase.hpp"
#include "imu/ImuBase.hpp"
#include "gps/GpsBase.hpp"
#include "magnetometer/MagnetometerBase.hpp"

namespace msr { namespace airlib {

class SensorCollection : UpdatableObject {
public: //types
    enum class SensorType : uint {
        Barometer = 1,
        Imu = 2,
        Gps = 3,
        Magnetometer = 4
    };
    typedef UpdatableContainer<SensorBase*> SensorBaseContainer;
public:
    void add(SensorBase* sensor, SensorType type)
    {
        auto type_int = static_cast<uint>(type);
        const auto& it = sensors_.find(type_int);
        if (it == sensors_.end()) {
            auto& pair = sensors_.emplace(type_int, std::make_shared<SensorBaseContainer>());
            pair.first->second->
        }
        else {
            it->second->insert(sensor);
        }
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {

    }

    virtual void update(real_T dt) override
    {

    }

    virtual void reportState(StateReporter& reporter) override
    {

    }
    //*** End: UpdatableState implementation ***//

private:
    unordered_map<uint, shared_ptr<UpdatableContainer<SensorBase*>>> sensors_;
};

}} //namespace
#endif
