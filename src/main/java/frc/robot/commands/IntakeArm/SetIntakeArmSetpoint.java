/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.IntakeArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SetIntakeArmSetpoint extends Command {
  protected double customAngleSetpoint;
  protected boolean isSetpointPersistant;
  public SetIntakeArmSetpoint(double angleSetpoint, boolean persistant) {
    if(angleSetpoint > RobotMap.intakeArmAbsoluteTop) {
      angleSetpoint = RobotMap.intakeArmAbsoluteTop;
      System.out.println("Arm setpoint set above physical MAXIMUM");
    }
    if(angleSetpoint < RobotMap.intakeArmAbsoluteBottom) {
      angleSetpoint = RobotMap.intakeArmAbsoluteBottom;
      System.out.println("Arm setpoint set below physical MINIMUM");
    }
    
    customAngleSetpoint = angleSetpoint;
    isSetpointPersistant = persistant;

    requires(Robot.m_intakeArm);
  }

  public boolean isOnTarget() {
    double angle = Robot.m_intakeArm.getAngle();
    if(  (angle < customAngleSetpoint+RobotMap.intakeArmAngleTolerance || customAngleSetpoint < angle-RobotMap.intakeArmAngleTolerance)  && angle <= RobotMap.intakeArmAbsoluteTop) {
      return true;
    } else {
      return false;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_intakeArm.runningCommand = true; //Signals it is running a command
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*double angle = Robot.m_intakeArm.getAngle(); //Gets angle in Deg
    System.out.println("Intake Arm angle:" + angle);
      if(  (angle < customAngleSetpoint+RobotMap.intakeArmAngleTolerance || customAngleSetpoint < angle-RobotMap.intakeArmAngleTolerance)  && angle <= RobotMap.intakeArmAbsoluteTop) {
        double motorSpeed = Math.signum(customAngleSetpoint - angle) * Robot.m_intakeArm.motionEquationLn( angle, customAngleSetpoint );

        Robot.m_intakeArm.setRawMotorSpeed(motorSpeed/2);
      } else {
        Robot.m_intakeArm.setRawMotorSpeed(0);
      } */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
    //return !Robot.m_intakeArm.runningCommand || (isOnTarget() && !isSetpointPersistant); //Returns false when currently running a command (so it doesn't stop)
    //Returns true if the robot is at the setpoint and is not persistant
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeArm.setRawMotorSpeed(0); //Stops the motor when the loop ends
    Robot.m_intakeArm.runningCommand = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
