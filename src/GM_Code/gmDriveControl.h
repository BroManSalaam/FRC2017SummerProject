
#pragma once

#include "WPILib.h"
#include <PowerDistributionPanel.h>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
using namespace std;
#include <chrono>
using namespace std::chrono;

namespace GM_Code
{

typedef duration<long, std::micro> duration_t;

class gmDriveControl
{
  public:
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Keep the complete list of values sent to the motors so
    // we can record a 'macro' and play it back later.
    //............................................................
    struct motors_s
    {
        high_resolution_clock::time_point when;
        float r, l;
        float yaw, enc_r, enc_l;
        long ts;

        motors_s(float arg_r, float arg_l) : when(high_resolution_clock::now()), r(arg_r), l(arg_l) {}
        motors_s(float arg_r, float arg_l, long arg_ts) : r(arg_r), l(arg_l), ts(arg_ts) {}

        motors_s(float arg_r, float arg_l, float arg_yaw) : when(high_resolution_clock::now()), r(arg_r), l(arg_l), yaw(arg_yaw) {}
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // (!) ... A similar constructor that adds arguments for
        //         setting encoder left/right can also be added.
        //............................................................
    };

  private:
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // These values can be used to 'tune' the drive's response
    // to the joystick. See gmDriveControl::addInput(). I suggest
    // not playing with them. It's pretty easy to go down a
    // rabbit hole with these that doesn't really improve the
    // driving experience.
    //............................................................
    const float p_min = 0.225f;
    const float p_mult = 2.0f;
    const float p_tmin = 0.25f;
    const float p_div = 5.0f;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We'll keep a running list of recent control inputs and
    // 'smooth' the motor outputs based on the inputs'
    // most-recently-weighted average.
    //............................................................
    struct pair_s
    {
        float r, l;
        float yaw, enc_r, enc_l;

        pair_s(float arg_r, float arg_l) : r(arg_r), l(arg_l) {}
    };
    deque<unique_ptr<pair_s>> dq;

    deque<shared_ptr<motors_s>> history_dq;
    std::map<int, shared_ptr<deque<shared_ptr<motors_s>>>> macros;

    float yaw_last;
    const int nInputs;
    std::unique_ptr<PowerDistributionPanel> pdp;
    double getMotorAdjustment();

  public:
    static mutex dspy_mutex;

    gmDriveControl(int n = 5) : yaw_last(0), nInputs(n), pdp(new PowerDistributionPanel()) {}

    void addInput(float x, float y, float z, float throttle, float yaw = 9999);
    bool getMotor(float &motorR, float &motorL);
    void setMotors(frc::RobotDrive* drive, float motorL, float motorR);
    void setMotors(frc::RobotDrive* drive, float motorL, float motorR, float yaw);
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // (!) ... And more stuff for passing the encode values, too.
    //............................................................

    void clear()
    {
        dq.clear();
        history_dq.clear();
    }

    bool saveMacro(int id, const high_resolution_clock::time_point &bgn, const high_resolution_clock::time_point &end);
    bool isMacro(int id);
    bool playMacro(int id, frc::RobotDrive* drive);

    shared_ptr<deque<shared_ptr<motors_s>>> getMacro(int id) { return macros[id]; }
    void putMacro(int id, shared_ptr<deque<shared_ptr<motors_s>>> macro) { macros[id] = macro; }
    vector<int> getMacros();

    static struct dspy_s
    {
        float throttle, turnR, turnL, spinR, spinL;
    } dspyValues;

    static dspy_s getDspyValues();
};

} // namespace GM_Code
