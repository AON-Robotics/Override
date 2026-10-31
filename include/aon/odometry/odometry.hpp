#pragma once

#ifndef AON_SENSING_ODOMETRY_HPP_
#define AON_SENSING_ODOMETRY_HPP_

#include <cmath>
#include <cstdint>
#include <iostream>
#include "../constants.hpp"
#include "../../api.h"
#if GYRO_ENABLED
#include "../../api.h"
#endif
#include "../tools/vector.hpp"
#include "../math/pose.hpp"
#include "heading-fusion.hpp"
#include "pose-estimator.hpp"


namespace aon {

    /**
     * \struct  ENCODER 
     *
     * \brief Store encoder data from current and previous odometry
     * iterations.
     * */
    struct ENCODER {
        double currentValue;
        double prevValue;
        double delta;
        double currentDistance;
        double previousDistance;
        double deltaDistance;

    };
    /**
     * \struct GYRO
     *
     * \brief Store gyro data from current and previous odometry
     * iterations.
     * */
    struct GYRO {
        double currentDegrees;
        double prevDegrees;
        double currentRadians;
        double deltaRadians;
        double deltaDegrees;

    };

    enum class SensorHealth {
        Unavailable, Baselining, Healthy, Invalid, Implausible, Stale
    };

    struct OdometryHealth {
        SensorHealth left = SensorHealth::Unavailable;
        SensorHealth right = SensorHealth::Unavailable;
        SensorHealth back = SensorHealth::Unavailable;
        SensorHealth motors = SensorHealth::Unavailable;
        SensorHealth leftMotors = SensorHealth::Unavailable;
        SensorHealth rightMotors = SensorHealth::Unavailable;
        SensorHealth imu = SensorHealth::Unavailable;
        SensorHealth otos = SensorHealth::Unavailable;
        SensorHealth gps = SensorHealth::Unavailable;
    };

    class Odometry {

    private:
        // Reversal flags for encoders
        bool leftReversed;
        bool rightReversed;
        bool backReversed;

        pros::Mutex pose_mutex;
        Pose currentPose;
        Pose rawPose;
        Pose rawOrigin;
        Pose fieldOrigin;
        HeadingFusion headingFusion;
        bool imuFusing = false;
        std::uint32_t lastPacketMs = 0;
        std::uint32_t lastEstimateMs = 0;
        bool hasPacket = false;
        bool originPending = true;
        bool sparkConsumed = false;
        bool hasTrackingSample = false;
        bool hadBackSample = false;
        bool hasMotorSample = false;
        bool wasSparkFresh = false;
        double previousLeft = 0;
        double previousRight = 0;
        double previousBack = 0;
        double previousMotorLeft = 0;
        double previousMotorRight = 0;
        double sparkOffsetX = 0;
        double sparkOffsetY = 0;
        double previousSparkHeading = 0;
        double fallbackHeading = 0;
        pros::MotorGroup* driveLeft = nullptr;
        pros::MotorGroup* driveRight = nullptr;
        int poseSource = 0;
        double traveledDistance = 0.0;
        OdometryHealth health;
        std::uint32_t lastUpdateMs = 0;
        bool hasUpdate = false;

        void resetCurrentLocked(double x, double y, double theta);
        bool readGps(Pose& result);

        bool acceptPose(const Pose& raw);

    public: 
        Odometry(short left, short right, short back, short gps, short gyro);
        Odometry(const Odometry& other);

        double getX();
        double getY();

        Vector getPosition();
        void SetPosition(double x, double y);

        /// Continuous degrees, retained for relative/multi-turn callers.
        double getDegrees();
        /// Clockwise heading normalized to [0, 360).
        double getHeading();
        /// Translational path length in inches, excluding corrections and turns.
        /// Resets on resetCurrent/resetInitial and pose setters (also GPS startup).
        double getTraveledDistance();
        /// Snapshot of input health; stale means the update/packet deadline expired.
        OdometryHealth getHealth();
        void setDegrees(double degrees);

        double getRadians();
        void setRadians(double radians);

        // MAIN functions
        void resetInitial();
        void initialize();
        void update();
        void resetCurrent(double x, double y, double theta);
        /// Valid GPS position in field inches; NaN coordinates when untrusted.
        Vector gpsPosition();
        /// One validated GPS startup attempt. Failure leaves the current pose intact.
        bool initializeFromGps();
        /// Consume one O,x,y,heading line (inches/clockwise degrees) from the Pi.
        /// Invalid input is rejected without refreshing packet freshness.
        bool ingestOtosLine(const char* line);
        Pose getPose();
        /// True while any position source is updating.
        bool hasFreshPose();
        bool hasFreshOtos();
        /// 0 none, 1 motor encoders, 2 tracking wheels, 3 OTOS, 4 both.
        int getPoseSource();
        void setDriveMotorGroups(pros::MotorGroup* left, pros::MotorGroup* right);
        void resetMotorBaselines();
        bool isImuFusing();
        double getOtosDegrees();


        //Debugging/Testing
        void debug();

    private:
        pros::Rotation encoderRight;
        pros::Rotation encoderLeft;
        pros::Rotation encoderBack;
        pros::Gps gps;

        #if GYRO_ENABLED
        pros::Imu gyroscope;
        #endif

    

    };
}
#endif
