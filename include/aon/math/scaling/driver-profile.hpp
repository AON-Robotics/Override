#pragma once

#include <memory>
#include <string>
#include "scaler.hpp"
#include "pilons-scaler.hpp"
#include "cubic-scaler.hpp"
#include "exponential-scaler.hpp"

namespace aon{
    enum class ScalerType{
        PILONS,
        CUBIC,
        EXPONENTIAL,
    };

    /// @brief A saved joystick scaling configuration for one driver.
    struct DriverProfile{
        std::string name;
        ScalerType curveType;
        double curveParam;
        double deadband;

        DriverProfile(std::string name, ScalerType curveType, double curveParam, double deadband) 
        : name(std::move(name)), curveType(curveType), curveParam(curveParam), deadband(deadband) {} 
    };

    /// @brief Builds a concrete Scaler instance from a profile's saved settings.
    inline std::unique_ptr<Scaler> buildScaler(const DriverProfile& profile){
        switch (profile.curveType){
            case ScalerType::CUBIC:
                return std::make_unique<CubicScaler>(profile.curveParam, profile.deadband);
            
            case ScalerType::EXPONENTIAL:
                return std::make_unique<ExponentialScaler>(profile.curveParam, profile.deadband);
            
            case ScalerType::PILONS:
            default:
                return std::make_unique<PilonsScaler>(profile.curveParam, profile.deadband);
        }
    }
    
}