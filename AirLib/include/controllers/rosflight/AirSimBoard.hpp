// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimRosFlightBoard_hpp
#define msr_airlib_AirSimRosFlightBoard_hpp

#include <exception>
#include "firmware/board.hpp"
#include "common/Common.hpp"
#include "vehicles/MultiRotorParams.hpp"
#include "sensors/SensorCollection.hpp"

//sensors
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"

namespace msr { namespace airlib {

class AirSimRosFlightBoard : public rosflight::Board {
public: //type
    typedef MultiRotorParams::EnabledSensors EnabledSensors;

public:
    AirSimRosFlightBoard(const EnabledSensors* enabled_sensors, const SensorCollection* sensors)
        : enabled_sensors_(enabled_sensors), sensors_(sensors)
    {
    }


    //interface implementation
    virtual void init() override 
    {
        imu = static_cast<const ImuBase*>(sensors_->getByType(SensorCollection::SensorType::Imu));

    }

    virtual uint64_t micros() override 
    {
        return static_cast<uint64_t>(Utils::getTimeSinceEpoch() * 1E6);
    }

    virtual uint32_t millis() override 
    {
        return static_cast<uint64_t>(Utils::getTimeSinceEpoch() * 1E3);
    }

    virtual void init_sensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback) override 
    {
    }

    virtual bool is_sensor_present(SensorType type) override 
    {
        switch (type) {
        case SensorType::Baro: return enabled_sensors_->barometer;
        case SensorType::Gps: return enabled_sensors_->gps;
        case SensorType::Imu: return enabled_sensors_->imu;
        case SensorType::Mag: return enabled_sensors_->magnetometer;
        default:
            return false;
        }
    }

    virtual uint16_t pwmRead(int16_t channel) override 
    {
    }

    virtual void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) override 
    {
    }

    virtual void pwmWriteMotor(uint8_t index, uint16_t value) override 
    {
    }

    virtual void set_led(uint8_t index, bool is_on) override 
    {
    }

    virtual void toggle_led(uint8_t index) override 
    {
    }

    virtual void init_params() override 
    {
    }

    virtual bool read_params() override 
    {
    }

    virtual bool write_params(bool blink_led) override 
    {
    }

    virtual void init_imu(uint16_t& acc1G, float& gyroScale, int boardVersion) override 
    {

    }

    virtual void read_accel(int16_t accel_adc[3]) override 
    {
        const auto& output = imu->getOutput();

        //Set acc1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
        uint16_t acc1G = 512 * 8;
        float accel_scale = 9.80665f/acc1G * 1.0f;

        accel_adc[0] = output.linear_acceleration.x() / 9.80665 * 512 * 8 / 1.0f;
    }

    virtual void read_gyro(int16_t gyro_adc[3]) override 
    {
    }

    virtual void read_temperature(int16_t& temp) override
    {
    }

    virtual void read_baro(float& altitude, float& pressure, float& temperature) override 
    {
    }

    virtual void read_diff_pressure(float& differential_pressure, float& temp, float& velocity) override 
    {
        throw std::exception("Diff pressure sensor is not available");
    }

    virtual float read_sonar() override 
    {
        throw std::exception("Sonar sensor is not available");
    }

    virtual void read_mag(int16_t mag_adc[3]) override 
    {
    }

    virtual void delay_micros(uint32_t us) override 
    {
    }

    virtual void delay_millis(uint32_t ms) override 
    {
    }

    virtual void system_reset(bool toBootloader) override 
    {
        throw std::exception("system reset is not implemented yet");
    }

private:
    uint16_t accel_to_adc(float accel)
    {
        return static_cast<uint16_t>(accel * accel_adc_bits_ / (accel_g_ * accel_scale_));
    }

private:
    const MultiRotorParams::EnabledSensors* enabled_sensors_;
    const SensorCollection* sensors_;
    const ImuBase* imu_;

    const uint16_t accel_adc_bits_ = 512 * 8; //for mpu6050 as per breezystm32/drv_mpu6050.c
    const float accel_scale_ = 1.0; //as set in PARAM_ACCEL_SCALE in ROSFlight
    const float accel_g_ = 9.80665f; //as set in ROSFlight sensors.c init_sensors() function
};

}} //namespace
#endif
