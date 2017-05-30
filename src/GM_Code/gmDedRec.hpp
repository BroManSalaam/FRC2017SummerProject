
#pragma once

#include "./GM_Code/gmCommon.h" // includes point_t and hvector_t
using namespace GM_Code;

#include "WPILib.h"
#include "AHRS.h" // navX-MXP

#include <cmath>
#include <deque>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
using namespace std;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// This module is just a first pass at getting a handle on the navX-MXP and
// its capabilities. The next step will probably require a couple of days
// measuring its performance when applying reasonable filtering techniques.
// OpenCV offers a Kalman Filter implementation that might be good to try:
//    #include <opencv2/core/core.hpp>
//    #include <opencv2/video/tracking.hpp>
//    cv::KalmanFilter KF;
// ----
// (!) But as the navX-MXP documentation says, errors in position accumulate
// to about 1 meter every 15 seconds. So it's likely that this device will
// be insufficient for any reasonable dead-reckoning, no matter what data
// refinement techniques are applied.
//................................................................................

namespace GM_Code
{
typedef duration<long, std::micro> duration_t;

class gmDedRec : public ITimestampedDataSubscriber
{
    struct zero_count_t
    {
        int x, y, z;
        zero_count_t() : x(0), y(0), z(0) {}
    } accZero;

    std::unique_ptr<AHRS> sensor_p{new AHRS(SPI::Port::kMXP, AHRS::kRawData, 200 /*samples/sec*/)}; // navX-MXP
    const float myEpsilon = 0.01f * 9.80665f;                                                       // 0.02 G is the navX-MXP default. (?)
    int calibrating;
    high_resolution_clock::time_point last_t;
    point_t refs, acc[2], vel[2], pos[2];
    float dt_avg;
    deque<unique_ptr<point_t>> positions;
    mutex positions_mutex;

    void waitForHdwrCalibration()
    {
        while (sensor_p->IsCalibrating())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void getAccel(point_t &acc, AHRSProtocol::AHRSUpdateBase &sensor_data)
    {
        // Values are in Gs: 9.80665 m/s^2. Convert to m/s^2.
        const float g = 9.80665f;
        acc.x = sensor_p->GetRawAccelX() * g;
        acc.y = sensor_p->GetRawAccelY() * g;
        acc.z = sensor_p->GetRawAccelZ() * g;

#if 0
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    	// Either these are raw sensor outputs or... what? I don't
    	// see them changing in my testing, but maybe they're just
    	// very small magnitude?
    	// (?) Maybe we can ask KauaiLabs about these.
        //............................................................
        acc.x = sensor_data.linear_accel_x;
        acc.y = sensor_data.linear_accel_y;
        acc.z = sensor_data.linear_accel_z;
#endif
    }

    void updatePosition(AHRSProtocol::AHRSUpdateBase &sensor_data, float dt)
    {
        hvector_t q(sensor_data.quat_x, sensor_data.quat_y, sensor_data.quat_z, sensor_data.quat_w);

        // acceleration (m/s^2)
        getAccel(acc[1], sensor_data);
        acc[1].x -= refs.x;
        acc[1].y -= refs.y;
        acc[1].z -= refs.z;

        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // I think we have to transform the acceleration vector by
        // the 'heading' using sensor_data.quat_{x,y,z,w} (?)
        //............................................................

        if (std::fabs(acc[1].x) < myEpsilon)
        {
            acc[1].x = 0;
            accZero.x += 1;
        }
        else
        {
            accZero.x = 0;
        }
        if (std::fabs(acc[1].y) < myEpsilon)
        {
            acc[1].y = 0;
            accZero.y += 1;
        }
        else
        {
            accZero.y = 0;
        }
        if (std::fabs(acc[1].z) < myEpsilon)
        {
            acc[1].z = 0;
            accZero.z += 1;
        }
        else
        {
            accZero.z = 0;
        }

#if 0
	    SmartDashboard::PutNumber("X ANGLE", (int)std::round(q.x * 1000));
	    SmartDashboard::PutNumber("Y ANGLE", (int)std::round(q.y * 1000));
	    SmartDashboard::PutNumber("Z ANGLE", (int)std::round(q.z * 1000));
	    SmartDashboard::PutNumber("W ANGLE", (int)std::round(q.w * 1000));

	    SmartDashboard::PutNumber("X ACCEL", (int)std::round(acc[1].x * 1000));
	    SmartDashboard::PutNumber("Y ACCEL", (int)std::round(acc[1].y * 1000));
	    SmartDashboard::PutNumber("Z ACCEL", (int)std::round(acc[1].z * 1000));
#endif

#if 1
        vel[1].x = vel[0].x + (acc[1].x + (acc[1].x - acc[0].x) / 2.0f) * dt;
        vel[1].y = vel[0].y + (acc[1].y + (acc[1].y - acc[0].y) / 2.0f) * dt;
        vel[1].z = vel[0].z + (acc[1].z + (acc[1].z - acc[0].z) / 2.0f) * dt;

        pos[1].x = pos[0].x + (vel[1].x + (vel[1].x - vel[0].x) / 2.0f) * dt;
        pos[1].y = pos[0].y + (vel[1].y + (vel[1].y - vel[0].y) / 2.0f) * dt;
        pos[1].z = pos[0].z + (vel[1].z + (vel[1].z - vel[0].z) / 2.0f) * dt;
#else
        // velocity (m/s)
        vel[1].x = vel[0].x + acc[1].x * dt;
        vel[1].y = vel[0].y + acc[1].y * dt;
        vel[1].z = vel[0].z + acc[1].z * dt;

        // position (m)
        pos[1].x = pos[0].x + vel[1].x * dt;
        pos[1].y = pos[0].y + vel[1].y * dt;
        pos[1].z = pos[0].z + vel[1].z * dt;
#endif

        // Are we really moving?
        vel[1].x = (accZero.x > 25) ? 0 : vel[1].x;
        vel[1].y = (accZero.y > 25) ? 0 : vel[1].y;
        vel[1].z = (accZero.z > 25) ? 0 : vel[1].z;

        acc[0].set(acc[1]);
        vel[0].set(vel[1]);
        pos[0].set(pos[1]);

#if 0
	    SmartDashboard::PutNumber("X VEL", std::round(vel[1].x * 100) / 100);
	    SmartDashboard::PutNumber("Y VEL", std::round(vel[1].y * 100) / 100);
	    SmartDashboard::PutNumber("X POS", std::round(pos[1].x * 100) / 100);
	    SmartDashboard::PutNumber("Y POS", std::round(pos[1].y * 100) / 100);
#endif

        unique_lock<mutex> lockit(positions_mutex);
        positions.push_back(unique_ptr<point_t>(new point_t(pos[1])));
        while (positions.size() > 2000) // Keep about 10 seconds of position samples.
            positions.pop_front();
    }

    void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSProtocol::AHRSUpdateBase &sensor_data, void *context) override
    {
        auto t = high_resolution_clock::now();
        duration_t dt = duration_cast<duration_t>(t - last_t);
        last_t = t;

        if (-1 == calibrating)
        {
            updatePosition(sensor_data, (float)dt.count() / 1000000.0f);
        }
        else
        {
            const int nsamps = 1024;
            point_t acc;

            dt_avg += (0 == calibrating) ? 0 : (float)dt.count();
            getAccel(acc, sensor_data);
            refs.x += acc.x;
            refs.y += acc.y;
            refs.z += acc.z;
            calibrating = (calibrating >= nsamps) ? -1 : calibrating + 1;

            if (-1 == calibrating)
            {
                refs.x /= (float)nsamps;
                refs.y /= (float)nsamps;
                refs.z /= (float)nsamps;

                char str[80];
                sprintf(str, "ded rec ready : refs = %f %f %f, sampling Hz = %d", refs.x, refs.y, refs.z,
                        (int)std::round(1.0f / ((dt_avg / (float)nsamps) / 1000000)));
                DriverStation::ReportError(str);
            }
        }
    }

  public:
    gmDedRec() : calibrating(-1), dt_avg(0)
    {
        waitForHdwrCalibration();

        DriverStation::ReportError("ded rec calibrating");
        sensor_p->ZeroYaw();
        calibrate();

        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Tell the navX to send new data as soon as it's available.
        // Values arrive at timestampedDataReceived() which overrides
        // a virtual method from the base class (from AHRS.h).
        //............................................................
        sensor_p->RegisterCallback(this, nullptr);
    }

    void calibrate()
    {
        calibrating = 0;
    }

    bool isMoving()
    {
        return std::fabs(vel[0].x) > myEpsilon || std::fabs(vel[0].y) > myEpsilon || std::fabs(vel[0].z) > myEpsilon;
    }

    void setPosition(point_t pos)
    {
        this->pos[0].x = pos.x;
        this->vel[0].x = this->acc[0].x = 0;
        this->pos[0].y = pos.y;
        this->vel[0].y = this->acc[0].y = 0;
        this->pos[0].z = pos.z;
        this->vel[0].z = this->acc[0].z = 0;

        positions.clear();
        positions.push_back(unique_ptr<point_t>(new point_t(this->pos[0])));
    }

    point_t getPosition()
    {
        unique_lock<mutex> lockit(positions_mutex);
        return (0 == positions.size()) ? point_t(0, 0, 0) : *positions.back();
    }

    float getYawAngle()
    {
        return sensor_p->GetYaw();
    }

    void resetYaw()
    {
    	sensor_p->ZeroYaw();
    }
};

} // namespace GM_Code
