/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.HID.HID;
import frc.robot.drive.DriveTrain;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

//MM: import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.ExampleCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();

  //public static HatchPickupManipulator m_hatchManipulator = new HatchPickupManipulator();
  public static Elevator m_elevator = new Elevator();
  public static Intake m_intake = new Intake();
  public static IntakeArm m_intakeArm = new IntakeArm();

  private HID m_HID = new HID();
  private DriveTrain m_driveTrain = new DriveTrain();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //MM: int mmRunTime = 0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    //CameraServer.getInstance().startAutomaticCapture(); //Start camera stream to dashboard

    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());

    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    m_driveTrain.initDefaultCommand();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    m_HID.init();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
//Drive Code
double driverJoystickX = m_HID.driveJoystick.getRawAxis(2)*-1;
double driverJoystickY = m_HID.driveJoystick.getRawAxis(1);
m_driveTrain.setRawPower(driverJoystickX, driverJoystickY);

//Manual Shooter
boolean intakeLz = m_HID.getAuxLzBumper();
boolean intakeRz = m_HID.getAuxRzBumper();
if(intakeLz) { //In
  m_intake.setRawSpeed(RobotMap.intakeMotorInSpeed);
}
if(intakeRz) { //Out
  m_intake.setRawSpeed(RobotMap.intakeMotorOutSpeed);
}
if(!(intakeLz || intakeRz)) { //If no input is detected, stop the motors
  m_intake.setRawSpeed(0);
}


if(m_HID.getAuxLBumper()) {
  double manualElevatorControl = -m_HID.getAuxYAxis(Hand.kLeft) * 0.5; //Note: May be inverted controls (up on joystick goes down, vice-versa)
  m_elevator.cancelCommand(); //Stop any running intakeArm command
  m_elevator.setManualControl(manualElevatorControl); //Runs at user speed
} else {
  m_elevator.setManualControl(0.11694561352299061);
}

if(m_HID.getAuxRBumper()) {
  //System.out.println("RBumper");
  double manualIntakeArmControl = m_HID.auxJoystick.getRawAxis(5); //Note: May be inverted controls (up on joystick goes down, vice-versa)
  m_intakeArm.cancelCommand(); //Stop any running intakeArm command
  m_intakeArm.setManualControl(manualIntakeArmControl); //Runs at user speed
}

  Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_HID.init();
    //MM code, ignore: driveTrain_subsystem.setMMRotationSetpoint(0); //Set the setpoint to the current motor position
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    
    //Drive Code
    double driverJoystickX = m_HID.driveJoystick.getRawAxis(2)*-1;
    double driverJoystickY = m_HID.driveJoystick.getRawAxis(1);
    double factor = SmartDashboard.getNumber("Factor", 0.5);
    m_driveTrain.setRawPower(driverJoystickX * factor, driverJoystickY * factor);

    //Manual Shooter
    boolean intakeLz = m_HID.getAuxLzBumper();
    boolean intakeRz = m_HID.getAuxRzBumper();
    if(intakeLz) { //In
      m_intake.setRawSpeed(RobotMap.intakeMotorInSpeed);
    }
    if(intakeRz) { //Out
      m_intake.setRawSpeed(RobotMap.intakeMotorOutSpeed);
    }
    if(!(intakeLz || intakeRz)) { //If no input is detected, stop the motors
      m_intake.setRawSpeed(0);
    }


    if(m_HID.getAuxLBumper()) {
      double manualElevatorControl = -m_HID.getAuxYAxis(Hand.kLeft) * 0.5; //Note: May be inverted controls (up on joystick goes down, vice-versa)
      m_elevator.cancelCommand(); //Stop any running intakeArm command
      m_elevator.setManualControl(manualElevatorControl); //Runs at user speed
    } else {
      m_elevator.setManualControl(0.11694561352299061);
    }
    
    if(m_HID.getAuxRBumper()) {
      //System.out.println("RBumper");
      double manualIntakeArmControl = m_HID.auxJoystick.getRawAxis(5); //Note: May be inverted controls (up on joystick goes down, vice-versa)
      m_intakeArm.cancelCommand(); //Stop any running intakeArm command
      m_intakeArm.setManualControl(manualIntakeArmControl); //Runs at user speed
    }

  /*MM code, ignore: { 
    double joystickY = m_HID.getYAxis(Hand.kLeft); // ^--v
    double joystickX = m_HID.getXAxis(Hand.kRight); // <-->
  
    if(m_HID.getRTrigger() > 0) { //Arcade mode when the user presses the right trigger
      driveTrain_subsystem.setRawPower(joystickX, joystickY);
    }
    if(m_HID.getAButton()) { //Motion Magic mode, rotates 6 rotations scaled by the joystick percentage
      driveTrain_subsystem.MMDrive(-joystickY);
    }
    if(m_HID.getYButton()) { //Resets the MM
      driveTrain_subsystem.zeroEncoders();
      driveTrain_subsystem.setMMRotationSetpoint(0); //Resets the setpoint by setting it to the current rotations
    }
    if(m_HID.getBButton()) { //Tries to move to the current MM setpoint
      driveTrain_subsystem.distanceMMDrive(1, 0.75); //1 rotation @ 0.75 speed
    }
    if(m_HID.getXButton()) {
      driveTrain_subsystem.setMMRotationSetpoint( SmartDashboard.getNumber("Set Target Rotations", 0.0) ); //Stores value in SD to the setpoint (Def value of 0 if key is not found (current position))
    }

    SmartDashboard.putNumber("Target Rotations", driveTrain_subsystem.getMMTargetDistanceSetpoint());
    SmartDashboard.putNumber("Target Angle", driveTrain_subsystem.getMMTargetAngleSetpoint());
  }*/
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
