/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.RobotMap;

public class SetElevatorSetpoint extends Command {
  protected double customHeightSetpoint;
  protected boolean isSetpointPersistant;
  /**
   * Sets the setpoint for the elevator and immediately runs
   * @param setpoint the setpoint in Inches
   * 
  */
  public SetElevatorSetpoint(double inches, boolean persistant) {
    System.out.println(inches);
    if(inches > RobotMap.elevatorTopPosition) {
      inches = RobotMap.elevatorTopPosition;
      System.out.println("Elevator setpoint set above physical MAXIMUM");
    }
    if(inches < RobotMap.elevatorBottomPosition) {
      inches = RobotMap.elevatorBottomPosition;
      System.out.println("Elevator setpoint set below physical MINIMUM");
    }
    
    customHeightSetpoint = inches - RobotMap.elevatorMountHeight; //Sets the set point to move the elevator subtracted by the height at which the elevator is mounted at
    isSetpointPersistant = persistant;

    Robot.m_elevator.runningCommand = true;
    requires(Robot.m_elevator);
  }

  public boolean isOnTarget() {
    double height = Robot.m_elevator.getElevatorHeight(); //Gets height in inches
    if(height < customHeightSetpoint+RobotMap.elevatorHeightTolerance || customHeightSetpoint < height-RobotMap.elevatorHeightTolerance) {
      return true;
    } else {
      return false;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_elevator.runningCommand = true; //Signals it is running a command
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Double.isNaN( Robot.m_elevator.getElevatorHeight() ) == true) { //Resets if the elevator goes past bottom set point (a small amount of weight can set this off or accelleration from the motor going down)
      Robot.m_elevator.resetEncoder();
    }

    double height = Robot.m_elevator.getElevatorHeight(); //Gets height in inches
    System.out.println("Elevator height" + height);
    System.out.print("running ");
    System.out.println(Robot.m_elevator.runningCommand);
      if(  (height < customHeightSetpoint+RobotMap.elevatorHeightTolerance || customHeightSetpoint < height-RobotMap.elevatorHeightTolerance)  && height <= RobotMap.elevatorTopPosition) {
        double motorSpeed = Math.signum(customHeightSetpoint - height) * Robot.m_elevator.motionEquationLn( height, customHeightSetpoint );
        System.out.println(motorSpeed);
        Robot.m_elevator.setRawMotorSpeed(motorSpeed);
      } else {
        Robot.m_elevator.setRawMotorSpeed(0);
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.print("IsFinished Called ");
    System.out.println(Robot.m_elevator.runningCommand);
    return !Robot.m_elevator.runningCommand; //|| (isOnTarget() && !isSetpointPersistant); //Returns false when currently running a command (so it doesn't stop)
                                                                                       //Returns true if the robot is at the setpoint and is not persistant
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevator.setRawMotorSpeed(0); //Stops the motor when the loop ends
    Robot.m_elevator.runningCommand = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
