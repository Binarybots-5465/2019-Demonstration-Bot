/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;


/**
 * Add your docs here.
 */
public class HatchPickupManipulator extends Subsystem {
  private VictorSP manipulatorMotor = new VictorSP(RobotMap.hatchManipulatorPWMID);
  private Counter motorCounter = new Counter(new DigitalInput(RobotMap.hatchManipulatorIntegratedCounterDIOID));
  private int hatchPosition = 0;
  
  /**
   * @return Updates and returns the current hatch position
   */
  public int updateHatchPosition() {
    hatchPosition += motorCounter.get() - hatchPosition; //Updates the hatch position from the last known value
    return hatchPosition;
  }

  /**
   * @return The hatch position
   */
  public int getHatchPosition() {
    return hatchPosition;
  }

  /**
   * @return true if the motor has hit the high bound
   */
  public boolean hasHitUpperBound() {
    if(hatchPosition >= RobotMap.hatchManipulatorHighPositionBound) 
      return true;
    else
      return false;
  }
  /**
   * @return true if the motor has hit the lower bound
   */
  public boolean hasHitLowerBound() {
    if(hatchPosition <= RobotMap.hatchManipulatorLowerPositionBound) 
      return true;
    else
      return false;
  }

  /**
   * @param vel The velocity to set the motor
   */
  public void setManipulatorMotorSpeed(double vel) {
    manipulatorMotor.set(vel);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
