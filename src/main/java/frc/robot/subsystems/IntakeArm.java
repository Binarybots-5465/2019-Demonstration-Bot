/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeArm extends Subsystem {
  protected Spark intakeArmMotorController = new Spark(RobotMap.intakeArmPWMID);
  protected AnalogPotentiometer intakeArmPot = new AnalogPotentiometer(RobotMap.intakeArmPotAIO, RobotMap.intakeArmRangeOfMotion, 0); //Potentiometer for arm control

  public boolean runningCommand = false;

  public double getAngle() {
    return intakeArmPot.get();
  }

  public void setRawMotorSpeed(double speed) {
    intakeArmMotorController.set(speed);
  }

  public void setManualControl(double speed) {
    double angle = intakeArmPot.get();
    if(  (angle <= RobotMap.intakeArmAbsoluteBottom + 2  && speed < 0) || (angle >= RobotMap.intakeArmAbsoluteTop - 2 && speed > 0)  ) { //Attempts to prevent the arm from moving outside the physical bounds
      setRawMotorSpeed(0);
      return;
    }
    setRawMotorSpeed(speed);
  }

  public void cancelCommand() {
    runningCommand = false;
  }

  public double motionEquationLn(double x, double angleSetpoint) {
    return Math.max( 0, Math.log( -1 * Math.signum(angleSetpoint - x) * (x - angleSetpoint) + 1)  /  Math.log(angleSetpoint + 1) );
    //If the angle is higher than the set point than the motor speed curve also flips the ln direction
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
