
#pragma once

#include "WPILib.h"

namespace GM_Code
{

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// gmJoystick
//........................................
class gmJoystick
{
    std::shared_ptr<Joystick> joystick;

  public:
    gmJoystick(int x) : joystick(new Joystick(x)) {}

    std::shared_ptr<Joystick> getJoystick()
    {
        return joystick;
    }

    int getJoystickButtons()
    {
        // Remember that button indexes begin at 1...
        int buttons = 0;
        for (auto i = 0; i < joystick.get()->GetButtonCount(); i++)
            buttons |= joystick.get()->GetRawButton(i + 1) ? (1 << i) : 0;
        return buttons;
    }

    int getJoystickButton(int button)
    {
    	return joystick.get()->GetRawButton(button);
    }

    int getJoystickPOV()
    {
        return joystick.get()->GetPOV();
    }

    void getJoystickValues(float &x, float &y, float &z, float &throttle)
    {
        x = joystick.get()->GetX();
        y = joystick.get()->GetY();
        z = joystick.get()->GetZ();
        throttle = joystick.get()->GetThrottle();
    }

    static int GetButtonBits(std::vector<int> buttons)
    {
        int bits = 0;
        // (!) Remember that button indexes begin at 1...
        for (int n : buttons)
            bits |= (1 << (n - 1));
        return bits;
    }
};

} // namespace GM_Code
