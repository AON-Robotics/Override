#pragma once

#include <cmath>
#include <algorithm>

namespace aon {

    /// @brief This abstract class is meant to be overridden by subclasses which 
    /// implement its `transform` method to transform analog inputs however the developer wishes. 
    /// The standard is that inputs come in [-127, 127] and outputs go in [-1, 1], 
    /// but this is not a hard set rule, although it is advised.
    class Scaler {
        protected:
            /// @brief Raw joysticks units below which the output is forced to 0
            double deadband;

        public:
            /// @param deadband The deadband radius in raw joystick units [0, 127]. Default 0 (no deadband).
            explicit Scaler(double deadband = 0.0) : deadband(std::max(0.0, deadband)){}

            virtual ~Scaler() = default;

            void setDeadband(double db) { deadband = std::max(0.0, db); }
            double getDeadband() const { return deadband; }

            /// @brief Applies the scaling strategy to a floating-point input.
            /// @param value The input value to transform.
            /// @return The transformed (scaled) value.
            virtual double transform(double value) = 0;

            double scale(double value){
                if(std::fabs(value) < deadband) return 0.0;
                return transform(value);
            }
    };

} // namespace aon