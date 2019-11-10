/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.HID;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * This extends the button class to accept axis so that it acts like a button (eg. Gamecube controller R & L triggers)
 */
public class AxisButton extends Button {
	protected GenericHID joy;
	protected int joystickAxis;
    public AxisButton(GenericHID joystick, int axis) {
		joy = joystick;
		joystickAxis = axis;
    }

    public boolean get() {
        return joy.getRawAxis(joystickAxis) > 0;
    }
}
