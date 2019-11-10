 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;

/**
 * This controls the elevator and should be able to set the elevator to a specific posistion
 */
public class Elevator extends Subsystem {
  
  private VictorSP elevatorMotorController1 = new VictorSP(RobotMap.elevatorLiftMotor1PWMID);
  private VictorSP elevatorMotorController2 = new VictorSP(RobotMap.elevatorLiftMotor2PWMID);

  public Encoder AMTEncoder = new Encoder(RobotMap.elevatorEncoderAChannelDIOID, RobotMap.elevatorEncoderBChannelDIOID); //Uses the AMT103-V encoder with inputs for A, B and index channel

  public boolean runningCommand = false;

  public Elevator() {
    //Resets the encoder and sets the PID source on startup
    AMTEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    AMTEncoder.reset();
    RobotMap.elevatorBottomPosition = getElevatorHeight(); //Sets the bottom distance to the elevator height at the bottom
  }

  public double getElevatorHeight() {
    return rawEncoderToInches(AMTEncoder.getDistance());
    //This returns the absolute POSITIVE distance that the motor has traveled (POSITIVE distance from reset point)
  }

  public double getRawTicks() {
    return AMTEncoder.getDistance(); //Get's the current height of the elevator in inches for setpoint
  }
  
  public void setRawMotorSpeed(double speed) {
    elevatorMotorController1.set(speed);
    elevatorMotorController2.set(speed);
  }

  public void setManualControl(double speed) {
    double height = getElevatorHeight();
    if(  (height <= RobotMap.elevatorBottomPosition + 1  && speed < 0) || (height >= RobotMap.elevatorTopPosition - 1 && speed > 0)  ) { //Prevents elevator from hitting the limits of the elevator
      setRawMotorSpeed(0);
      return;
    }
    setRawMotorSpeed(speed);
  }

  public void resetEncoder() {
    AMTEncoder.reset();
  }

  public void cancelCommand() {
    System.out.println("Canceled elevator Command");
    runningCommand = false;
  }

  protected static double inchesToRawEncoder(double inches) {
    double correctedInches = 3584.81*Math.pow(-1 - 4217.87/(-4227.24 + inches), 0.980567120799986); //Undo's the regression converting the distance correction
    return correctedInches * RobotMap.AMTEncoderResolution / (2 * Math.PI * RobotMap.elevatorOverallDriveRatio) / 100; //Converts to from inches to rotation ticks
  }

  protected static double rawEncoderToInches(double encoderTicks) {
    double x = 100 * encoderTicks * 2 * Math.PI * RobotMap.elevatorOverallDriveRatio / (RobotMap.AMTEncoderResolution); //Converts from encoder ticks and multiplies to get to inches
    return 4227.24 + (9.371548 - 4227.24)/( 1 + Math.pow(x/3584.812, 1.019818) ); // Corrects result from above to the correct absolute value accounting the inital starting height of the mechanism.
                                                                                  // Bottom position ~ 9.3 inches instead of bottom position = 0.0 inches (The bottom resting position takes account for the frame). 
                                                                                  // Equation used for correction wass taken from a Symmetrical Sigmoidal regression of point made from the original rotation-distance conversion and the actual height distances.
  }

  public double motionEquationSqrt(double x, double heightSetpoint) {
    return Math.sqrt( -1 * Math.signum(heightSetpoint - x) * (x - heightSetpoint) )  /  Math.sqrt(heightSetpoint);
    //If the height is higher than the set point than the motor speed curve also flips the sqrt direction
  }

  public double motionEquationLn(double x, double heightSetpoint) {
    return Math.max( 0, Math.log( -1 * Math.signum(heightSetpoint - x) * (x - heightSetpoint) + 1)  /  Math.log(heightSetpoint + 1) );
    //If the height is higher than the set point than the motor speed curve also flips the sqrt direction
  }

  @Override
  public void initDefaultCommand() {
  }
}
