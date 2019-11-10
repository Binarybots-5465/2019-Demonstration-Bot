/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.RobotMap;
import frc.robot.Robot;

public class SetIntakeIn extends Command {
  public SetIntakeIn() {
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_intake.runningCommand = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intake.setRawSpeed(RobotMap.intakeMotorInSpeed); //Set speed to go suck ball in

    if(Robot.m_intake.checkLimitSwitch() == true) { //The ball was caught, stop the command
      end(); //End the command
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.m_intake.runningCommand; //Returns false when currently running a command (so it doesn't stop)
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.setRawSpeed(0);
    Robot.m_intake.runningCommand = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
