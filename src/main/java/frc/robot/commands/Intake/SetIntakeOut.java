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

public class SetIntakeOut extends Command {

  protected long ballOutTime = 0; //Inital value
  public SetIntakeOut() {
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_intake.runningCommand = true;
    ballOutTime = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(ballOutTime == 0 || (System.currentTimeMillis() - ballOutTime)/1000.0d < RobotMap.intakeOutMotorTimeout) { //Time elapsed since the ball has been out = 0 (hasn't left) or is less then the time point
      Robot.m_intake.setRawSpeed(RobotMap.intakeMotorOutSpeed);  //Set speed to push ball out
    } else {
      end(); //Stops command
    }

    if(Robot.m_intake.checkLimitSwitch() == false && ballOutTime == 0) { //Sets time setpoint when ball isn't being detected by switch & timer hasn't already started
      ballOutTime = System.currentTimeMillis(); //Sets the time setpoint
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
