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

    class Odometry {

    private:
        double deltaTheta;
        Vector deltaDlocal;
        Angle orientation;
        Vector position;
        Vector changeWeb;
        const double conversionFactor;

        ENCODER encoderBack_data;
        ENCODER encoderRight_data;
        ENCODER encoderLeft_data;
        GYRO gyro_data;

        // Reversal flags for encoders
        bool leftReversed;
        bool rightReversed;
        bool backReversed;

        pros::Mutex p_mutex;
        pros::Mutex orientation_mutex;
        pros::Mutex pose_mutex;
        Pose currentPose;
        Pose rawPose;
        Pose rawOrigin;
        Pose fieldOrigin;
        HeadingFusion headingFusion;
        bool imuFusing = false;
        std::uint32_t lastPacketMs = 0;
        bool hasPacket = false;
        bool originPending = true;

        void acceptPose(const Pose& raw);

    public: 
        Odometry(short left, short right, short back, short gps, short gyro);
        Odometry(const Odometry& other);

        double getX();
        double getY();

        Vector getPosition();
        void SetPosition(double x, double y);

        double getDegrees();
        void setDegrees(double degrees);

        double getRadians();
        void setRadians(double radians);

        // MAIN functions
        void resetInitial();
        void initialize();
        void update();
        void resetCurrent(double x, double y, double theta);
        Vector gpsPosition();
        Pose getPose();
        /// Valid only while OTOS packets arrive within 300 ms.
        bool hasFreshPose();
        bool isImuFusing();
        double getOtosDegrees();


        //Debugging/Testing
        void debug();

        pros::Rotation encoderRight;
        pros::Rotation encoderLeft;//was in private 
        pros::Rotation encoderBack;
        pros::Gps gps;

        #if GYRO_ENABLED
        pros::Imu gyroscope;
        #endif

    

    };
}
#endif
