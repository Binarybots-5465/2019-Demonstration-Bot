/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import frc.robot.RobotMap;
import frc.robot.commands.Elevator.SetElevatorSetpoint;
import frc.robot.commands.Intake.SetIntakeOut;
import frc.robot.commands.IntakeArm.SetIntakeArmSetpoint;

public class ElevatorLevelThreeCG extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ElevatorLevelThreeCG() {
    addParallel(new SetElevatorSetpoint(RobotMap.elevatorLevelTop, true)); //Moves the elevator to the 3rd level and holds it there until the meathod cancelCommand() is called from m_intakeArm or another command using m_intakeArm is called
    addParallel(new SetIntakeArmSetpoint(RobotMap.intakeArmUprightPosition, true)); //Sets arm upright and holds it there until canceled or interupted

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
