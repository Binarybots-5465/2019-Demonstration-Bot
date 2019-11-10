/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.HID;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.HID.AxisButton;
import frc.robot.commands.CommandGroups.ElevatorLevelThreeCG;
import frc.robot.commands.CommandGroups.ElevatorLevelTwoCG;
import frc.robot.commands.CommandGroups.IntakeArmFeedBallCG;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.CommandGroups.*;


/**
 * Add your docs here.
 */
public class HID {
  public Joystick driveJoystick;
  public Joystick auxJoystick;
  
  private DriverStation ds = DriverStation.getInstance();

  //vvv Note: Using manual control of intake mechanism vvv
  //private Button shooterInButton;
  //private Button shooterOutButton;
  private JoystickButton cargoShipPosition;
  private JoystickButton firstLevelHatchPosition;
  private JoystickButton secondLevelHatchPosition;
  private JoystickButton topLevelHatchPosition;

  public void init() {
    int foundJoysticks = 0;
    for(int port = 0; port <= 5; port++) {
      String jName = ds.getJoystickName(port);
      System.out.println("Registered DS Joystick: " + jName);

      if(jName.equals(RobotMap.driveJoystickName)) { //Found Driver Joystick
        driveJoystick = new Joystick(port);
        foundJoysticks++;
        System.out.println("Found Drive Joystick: " + jName + " on port " + port);
      }
      if(jName.equals(RobotMap.auxJoystickName)) { //Found Aux Joystick
        auxJoystick = new Joystick(port);
        foundJoysticks++;
        System.out.println("Found Aux Joystick: " + jName + " on port " + port);
      }
    }

    if(foundJoysticks < 2) {
      System.out.println("Only found " + foundJoysticks + " joysticks!");
      System.out.println("Make sure you can see both (" + RobotMap.driveJoystickName + ") and (" + RobotMap.auxJoystickName + ") in the driverstation!");
    }

    //vvv Note: Using manual control of intake mechanism vvv
    //shooterInButton = new AxisButton(auxJoystick, 2); //Bottom L Trigger (Axis # 2)
    //shooterOutButton = new AxisButton(auxJoystick, 3); //Bottom R bumper (Axis # 3)
    cargoShipPosition = new JoystickButton(auxJoystick, 1); //A
    firstLevelHatchPosition = new JoystickButton(auxJoystick, 2); //B
    secondLevelHatchPosition = new JoystickButton(auxJoystick, 4); //Y
    topLevelHatchPosition = new JoystickButton(auxJoystick, 3); //X

    //vvv Note: Using manual control of intake mechanism vvv
    //shooterInButton.whenPressed(new SetIntakeIn());
    //shooterOutButton.whenPressed(new SetIntakeOut());
    
    cargoShipPosition.whenPressed(new SetElevatorSetpoint( RobotMap.elevatorCargoShip, true ));
    firstLevelHatchPosition.whenPressed(new SetElevatorSetpoint( RobotMap.elevatorLevelBottom, true ));
    secondLevelHatchPosition.whenPressed(new SetElevatorSetpoint( RobotMap.elevatorLevelMiddle, true ));
    topLevelHatchPosition.whenPressed(new SetElevatorSetpoint( RobotMap.elevatorLevelTop, true ));
  }

  public void setSensitivity(double sensitivity) {
    RobotMap.sensitivityCoefficient = sensitivity;
  }

  protected double adjustForSensitivity(double value) { 
    value = RobotMap.sensitivityCoefficient * Math.pow(value, 3) + (1 - RobotMap.sensitivityCoefficient) * value;

    if(value > -RobotMap.joystickDeadband && value < RobotMap.joystickDeadband)
      value = 0;
    
    return value;
  }

  public double getDriverYAxis(GenericHID.Hand hand) {
    return adjustForSensitivity( driveJoystick.getY(hand) );
  }
  public double getDriverXAxis(GenericHID.Hand hand) {
    return adjustForSensitivity( driveJoystick.getX(hand) );
  }

  public double getAuxYAxis(GenericHID.Hand hand) {
    return adjustForSensitivity( auxJoystick.getY(hand) );
  }
  public double getAuxXAxis(GenericHID.Hand hand) {
      return adjustForSensitivity( auxJoystick.getX(hand) );
  }

  public boolean getAuxLzBumper() {
    return auxJoystick.getRawAxis(2) > 0;
  }

  public boolean getAuxRzBumper() {
    return auxJoystick.getRawAxis(3) > 0;
  }

  //X-O-Sq-Tri
  public boolean getDriverXButton() {
    return driveJoystick.getRawButton(2);
  }
  public boolean getDriverOButton() {
    return driveJoystick.getRawButton(3);
  }
  public boolean getDriverSquareButton() {
    return driveJoystick.getRawButton(1);
  }
  public boolean getDriverTriangleButton() {
    return driveJoystick.getRawButton(4);
  }

  //ABXY
  public boolean getAuxAButton() {
    return auxJoystick.getRawButton(1);
  }
  public boolean getAuxBButton() {
    return auxJoystick.getRawButton(2);
  }
  public boolean getAuxXButton() {
    return auxJoystick.getRawButton(3);
  }
  public boolean getAuxYButton() {
    return auxJoystick.getRawButton(4);
  }

  public boolean getAuxLBumper() {
    return auxJoystick.getRawButton(5);
  }

  public boolean getAuxRBumper() {
    return auxJoystick.getRawButton(6);
  }
  
/*
  public boolean getAButton() {
    return xboxJoystick.getAButton();
  }

  public boolean getBButton() {
    return xboxJoystick.getBButton();
  }

  public boolean getYButton() {
    return xboxJoystick.getYButton();
  }

  public boolean getXButton() {
    return xboxJoystick.getXButton();
  }

  public double getRTrigger() {
    return xboxJoystick.getTriggerAxis(GenericHID.Hand.kRight);
  }

  public double getLTrigger() {
    return xboxJoystick.getTriggerAxis(GenericHID.Hand.kLeft);
  }*/
}
