
#include "gmDriveControl.h"
#include "gmCommon.h"

#include <PowerDistributionPanel.h>

// See Robot.cpp for this function:
extern bool isButtonChange();

namespace GM_Code
{

gmDriveControl::dspy_s gmDriveControl::dspyValues;
mutex gmDriveControl::dspy_mutex;

gmDriveControl::dspy_s gmDriveControl::getDspyValues()
{
    unique_lock<mutex> lockit(dspy_mutex);
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // A new copy of dspyValues will be created and returned,
    // and lockit will be destroyed, releasing dspy_mutex.
    //............................................................
    return dspyValues;
}

void gmDriveControl::addInput(float x, float y, float z, float throttle, float yaw)
{
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Okay, joystick x is touchy so we do need a dead spot.
	// (Try to make it as small as possible.)
    //............................................................
	float deadRange = 0.35; // Maybe too large?
	if (x < -deadRange) x = (x - -deadRange) / (1.0 - deadRange); else if (x > deadRange) x = (x - deadRange) / (1.0 - deadRange); else x = 0;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // 'y' and 'throttle' values from the joystick decrease when
    // the stick is pushed forward and increase when it's
    // pulled back. Both with a range of -1 to +1.
    //............................................................

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Remap the throttle position to: min = p_min, max = 1.0.
    // Note that on my 'peanut' robot, ~0.225 is the minimum
    // usable throttle that will drive a slow spin move.
    //............................................................
    throttle = 1.0f - (throttle + 1.0f) / (2.0f / (1.0f - p_min));

    // Speed smoothly varies between 0 -> 1, or 0 -> -1...
    float speed = ((y < 0) ? -smootherstep(0, 1, -y) : smootherstep(0, 1, y)) * throttle;
    // ... Likewise for spinR and spinL.
    float spinR = ((z < 0) ? -smootherstep(0, 1, -z) : smootherstep(0, 1, z)) * ((throttle - p_min) / p_div + p_min);
    float spinL = -spinR;
    // TurnR/L smoothly vary between p_tmin -> 1.
    float turnR = (x - (-1.0f)) / 2.0f;
    turnR = 1.0f - smootherstep(0, 1, turnR);
    turnR = lerp(p_tmin, 1, turnR);
    float turnL = (x + 1.0f) / 2.0f;
    turnL = smootherstep(0, 1, turnL);
    turnL = lerp(p_tmin, 1, turnL);

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Apply a correction based on yaw, but just for 'straight'
    // driving.
    //............................................................
	if (fabs(turnL - turnR) < 0.15 && fabs(speed) > 0.1)
	{
	    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// (!) Tune the amount value. Just below where oscillations
		// occur because of over-correction is probably good.
	    //............................................................
		const float amount = 0.40; // Tune this value!
		float dy = (yaw - this->yaw_last) / 1.0 * amount; // ~ degrees-per-20ms * amount
		dy = (dy < -1) ? -1 : ((dy > 1) ? 1 : dy);

		// (!!!) Depends on drive motor directions!
		if (speed > 0)
		{
			if (dy < 0) turnL -= turnL * -dy; else turnR -= turnR * dy;
		}
		else
		{
			if (dy < 0) turnR -= turnR * -dy; else turnL -= turnL * dy;
		}
		this->yaw_last = yaw;
	}

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Use the magnitude of the joystick 'direction' to normalize
    // the joystick inputs...
    //............................................................
    float tMag = sqrt(turnR * turnR + turnL * turnL);
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ... But use the magnitude of the joystick 'twist' to
    // mix-in the spin component.
    //............................................................
    float sMag = sqrt(spinR * spinR + spinL * spinL) * p_mult;
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Note that, at rest, turnR/L are both 0.5; so tMag really
    // varies only between 0.707 and 1.0.
    //............................................................
    float motorR = lerp(speed * turnR / tMag, spinR, sMag);
    float motorL = lerp(speed * turnL / tMag, spinL, sMag);

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Append the new motor inputs to our list...
    //............................................................
    dq.push_back(unique_ptr<pair_s>(new pair_s(motorR, motorL))); // See footnote (A).
    // ... And make sure the list doesn't get too long.
    while ((int)dq.size() > nInputs)
        dq.pop_front();

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Keep track of the most recent inputs so we can visualize
    // them on the driver console.
    //............................................................

    // Use dspy_mutex to identify this 'critical section' of code:
    unique_lock<mutex> lockit(dspy_mutex);
    dspyValues.throttle = throttle;
    dspyValues.turnR = turnR;
    dspyValues.turnL = turnL;
    dspyValues.spinR = spinR;
    dspyValues.spinL = spinL;
    // lockit is destroyed here, releasing dspy_mutex.
}

bool gmDriveControl::getMotor(float &motorR, float &motorL)
{
    int len = 1, total = 0;

    motorR = motorL = 0;
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Iterate over each member of our deque, summing the motor
    // inputs with more weight for the more recent.
    //............................................................
    for (unique_ptr<pair_s> &item : dq) // See footnote (B).
    {
        motorR += item->r * len;
        motorL += item->l * len;
        total += len;
        len += 1;
    }

    if (total > 0)
    {
        // Compute the recently-weighted average.
        motorR /= total;
        motorL /= total;
        return true;
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Return false if we have no data.
    //........................................
    return false;
}

double gmDriveControl::getMotorAdjustment()
{
    // (!!!) GetTotalCurrent() times-out and fails. (?)
    // double A_total = pdp->GetTotalCurrent();

    double V_in = pdp->GetVoltage();
    double Temp = pdp->GetTemperature(); // Might come in handy. (?)
    // For each drive motor...
    double A_1 = pdp->GetCurrent(1); // front left
    double A_0 = pdp->GetCurrent(0); // rear left
    double A_2 = pdp->GetCurrent(2); // front right
    double A_3 = pdp->GetCurrent(3); // rear right

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Eventually, I think, we'll set up a machine-learning (ML)
    // method where we use these inputs along with the
    // (instantaneous) velocity and delta-yaw coming back from
    // the navX to train an ML algorithm to find adjustments.
    // (!) Meanwhile we'll just use these in some meaningless
    // equation to eliminate the 'unused variable' warnings
    // from the compiler.
    //............................................................
    double a = 1; /*V_in + (A_1 + A_0) / 2 + (A_2 + A_3) / 2 + Temp;*/
    return a / a; // Return 1.0.
}

void gmDriveControl::setMotors(frc::RobotDrive* drive, float motorL, float motorR)
{

	//motorL = -motorL;
    //motorR = -motorR;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Store the most recent 15000 motor inputs. (~5 minutes)
    //............................................................
    while ((int)history_dq.size() > 15000)
        history_dq.pop_front();

    history_dq.push_back(unique_ptr<motors_s>(new motors_s(motorR, motorL)));

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Here is where we'll adjust the motor inputs, varying them
    // according to battery voltage and current.
    //............................................................
    float a = getMotorAdjustment();

    drive->SetLeftRightMotorOutputs(motorL * a, motorR * a);
}

void gmDriveControl::setMotors(frc::RobotDrive* drive, float motorL, float motorR, float yaw)
{
    //motorL = -motorL;
    //motorR = -motorR;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Store the most recent 15000 motor inputs. (~5 minutes)
    //............................................................
    while ((int)history_dq.size() > 15000)
        history_dq.pop_front();

    history_dq.push_back(unique_ptr<motors_s>(new motors_s(motorR, motorL, yaw)));

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Here is where we'll adjust the motor inputs, varying them
    // according to battery voltage and current.
    //............................................................
    float a = getMotorAdjustment();

    drive->SetLeftRightMotorOutputs(motorL * a, motorR * a);
}

bool gmDriveControl::saveMacro(int id, const high_resolution_clock::time_point &bgn, const high_resolution_clock::time_point &end)
{
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Note that for std::map, the insert method will fail if the
    // key already exists, the [] operator will update the
    // element with a new value, or create a new one.
    //............................................................

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Create a new macro by copying all 'controls' that occurred
    // between the bgn and end time_points to a new deque.
    //............................................................
    shared_ptr<deque<shared_ptr<motors_s>>> macro(new deque<shared_ptr<motors_s>>);

    for (shared_ptr<motors_s> &item : history_dq)
    {
        duration_t ts_1 = duration_cast<duration_t>(item->when - bgn);
        duration_t ts_2 = duration_cast<duration_t>(end - item->when);

        if (ts_1 > std::chrono::duration_values<duration_t>::zero() && ts_2 > std::chrono::duration_values<duration_t>::zero())
            macro->push_back(item);
    }

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Trim the new macro by removing all leading and
    // trailing control values that are (almost) zero.
    //............................................................
    const float motorEpsilon = 0.005f;
    int removeBgn = 0, removeEnd = 0;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Determine how many to remove at the front and back...
    //............................................................
    auto it = macro->begin();
    while (it != macro->end())
    {
        if (fabs((*it)->l) < motorEpsilon && fabs((*it)->r) < motorEpsilon)
            removeBgn++;
        else
            break;
        it++;
    }
    it = macro->end();
    while (it != macro->begin())
    {
        it--;
        if (fabs((*it)->l) < motorEpsilon && fabs((*it)->r) < motorEpsilon)
            removeEnd++;
        else
            break;
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ... Then remove them.
    //............................................................
    for (auto i = 0; i < removeBgn; i++)
        macro->pop_front();
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // If no real data was recorded, we'll know right here. (!)
    //............................................................
    if (0 == macro->size())
        return true;

    for (auto i = 0; i < removeEnd; i++)
        macro->pop_back();

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Lastly, pre-process the time 'offsets' so we don't have
    // to mess with them later.
    //............................................................
    high_resolution_clock::time_point tLast = (*macro)[0]->when;
    for (shared_ptr<motors_s> &item : *macro)
    {
        auto time_span = duration_cast<duration_t>(item->when - tLast);

        item->ts = time_span.count();
        tLast = item->when;
    }

    macros[id] = macro;
    return false;
}

bool gmDriveControl::isMacro(int id)
{
    return macros.count(id); // Can only return 0 or 1.
                             //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                             // Hey, students! Unlike some other languages, a bool in C/C++ is really just
                             // something that evaluates to zero (false) or non-zero (true). So there's a long
                             // tradition in C/C++ of not worrying about 'bool' as a real type.
                             // In JavaScript, for example, the above would be written something like:
                             //    return macros.count(id) != 0;
                             // This provides the necessary type conversion to 'bool', as well as more clarity.
                             //................................................................................
}

bool gmDriveControl::playMacro(int id, frc::RobotDrive* drive)
{
    if (!isMacro(id))
        return false;

    shared_ptr<deque<shared_ptr<motors_s>>> macro = macros[id];

    for (shared_ptr<motors_s> &item : *macro)
    {
        // Terminate macro if a button is pressed.
        if (isButtonChange())
            break;

        std::this_thread::sleep_for(std::chrono::microseconds((int)item->ts));

        drive->SetLeftRightMotorOutputs(item->l, item->r);
    }
    return true;
}

vector<int> gmDriveControl::getMacros()
{
    vector<int> list;

    for (auto m : macros)
    {
        list.push_back(m.first);
    }
    return list;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Footnotes:
//
// (A) Unlike Java, C#, Javascript, etc., C++ does not do automatic memory
//     management (aka, garbage collection). But several 'smart' pointer types
//     are commonly used to implement something nearly as good.
//
//     So, a list (deque) of unique_ptrs was defined--instead of a list of
//     normal pointers--because 'delete' will automatically be called when each
//     of the pair_s objects (we create at (A)) is later removed from the
//     list (by 'dq.pop_front();' a couple of lines later).
//
// (B) C++11 is required for this slick style of iterator to compile.
//     That's okay because g++ is C++11 (or C++14, actually) compliant.
//     You'll see the argument '-std=c++1y' passed to the compiler by Eclipse,
//     which really just means "use the C++11 features".
//................................................................................

} // namespace GM_Code
