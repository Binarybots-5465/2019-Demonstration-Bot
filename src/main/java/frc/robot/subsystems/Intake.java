/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {

  protected Spark intakeMotorController;
  protected DigitalInput intakeLimitSwitch = new DigitalInput(RobotMap.intakeLimitSwitchDIO);
  
  public boolean runningCommand = false;

  public Intake() {
    intakeMotorController = new Spark(RobotMap.intakeMotorPWMID);
  }

  public void setRawSpeed(double speed) {
    intakeMotorController.set(speed);
  }

  public boolean checkLimitSwitch() {
    return intakeLimitSwitch.get(); //Checks if a ball is in side the shooter (0 = not pressed (not ball), 1 = pressed (ball))
  }

  public void cancelCommand() {
    runningCommand = false;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
