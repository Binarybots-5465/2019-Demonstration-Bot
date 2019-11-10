/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.drive.PIDGains;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  /* Drive Constants */

    //Talon ID's
    public static final double driveDeadband = 0.01; //1% Deadband.
    public static int[] rightTalonID = new int[]{1, 2}; //Encoder on 1
    public static int[] leftTalonID = new int[]{3, 4}; //Encoder on 3

    //Encoder Positions
    public static int leftTalonEncoderID = 1; //Talon ID that the encoder is on
    public static int rightTalonEncoderID = 3;
    
    public static double encoderUnitsPerRotation = 4096; //4096 units per rotation given by encoder. (per 100ms)
    public static int talonEncoderTimeout = 30; //100 ms timeout for encoder to respond
    
    public static double driveTrainWheelDiameter = 6; //6 inch wheel diameter

    //Motion Magic Constants & Gains
    public static double motorPeakVelocity = 5655; //(kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio)
    public static int initialCruiseVelocity = (int)( motorPeakVelocity * 3/4 ); // 75% top speed
    public static int initialCruiseAcceleration = (int)( motorPeakVelocity * 3/4 ); //If initialCruiseAcceleration == initialCruiseVelocity it will take 1 second to get up to speed
    
    /**
      * PID Gains may have to be adjusted based on the responsiveness of control loop.
      * kF: 1023 represents output value to Talon at 100%
      * Note: I calculated gain values from a tool I found on CD, I copied it to my personal Google drive (https://docs.google.com/spreadsheets/d/1c2VJxmdmWaVfT8U2ppdRMaNbdX99FHWba4rvoxasips/edit?usp=sharing)
      * 	                                    			           kP   kI   kD    kF             Iz   PeakOut */
    public final static PIDGains distanceGains = new PIDGains( 0.0, 0.0,  0.0, 0.1809018568,  100,  0.50 );
    public final static PIDGains turningGains =  new PIDGains( 0.0, 0.0,  0.0, 0.1809018568,  200,  1.00 );

    //Sets constants for PID Index
    // We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary
    public final static int PIDPrimary = 0;
    public final static int PIDTurn = 1;
    
    //Sets constants for PID remote sensors
    // We allow either a 0 or 1 when selecting an ordinal for remote devices (You can have up to 2 devices assigned remotely to a talon/victor)
    public final static int PIDRemote0 = 0;
    public final static int PIDRemote1 = 1;

    //Sets constants for PID slots used for distance and turning
    // Firmware supports slots [0, 3] and can be used for either PID Set
    public final static int PIDDistanceSlot = 0;
    public final static int PIDTurningSlot = 1;

    //Sets constant for the motor deadband for PID
    public final static double PIDNeutralDeadband = 0.04; // 4% deadband

    //Sets constant for the length of time the PID period runs
    public final static int PIDClosedLoopTimeMs = 1; //1 ms per loop cycle

  /* Elevator Constants */

    //Sets elevator PWM slot ID
    public static int elevatorLiftMotor1PWMID = 0;
    public static int elevatorLiftMotor2PWMID = 1;

    //Sets channels for the elevator encoder slots
    public static int elevatorEncoderAChannelDIOID = 1;
    public static int elevatorEncoderBChannelDIOID = 2;
    public static int elevatorEncoderIndexChannelDIOID = 3;

    //Sets constants for PID
    public static double elevatorHeightTolerance = 0.1; //The maximum in which the elevator returns true if it's within the tolerance

    //Sets constant for the AMT103-V encoder resolution
    public static double AMTEncoderResolution = 2048; //This can be set on the encoder (all DIP-switches are down = 2048 pulses per rot)
    
    //Sets constant for the overall gear ratio
    public static double elevatorOverallDriveRatio = 1/74.8741298717144; //Speed ratio, check Elevator Calculator for more detail (https://docs.google.com/spreadsheets/d/1Esn9BTLFzwUF1rX5MHBDLCF1DLpibluD17OtHajhIUg/edit)
    
    //Sets the height of the elevator mount point so that the height the elevator is set to is corrected (eg. when set to 50 inches, the top of the elevator mount is at 50 inches above THE GROUND [not 50in above the robot frame])
    public static double elevatorMountHeight = 2.545; //2.545 Inches from bottom of ground to bellypan (mountpoint)
    
    //Sets constants for the height and movement limits for the elevator (Inches)
    public static double elevatorBottomPosition = 9.371548 + elevatorMountHeight; //The height of the elevator is
    public static double elevatorTopPosition = 70 + elevatorBottomPosition; //Absolute height from ground to top of the robot elevator
    

    //Sets the sign of the direction of the elevator for going up & down
    public static double elevatorUpSign = 1;
    public static double elevatorDownSign = -1;

    public static double elevatorCargoShip = 17.0;
    public static double elevatorLevelBottom = RobotMap.elevatorBottomPosition; //Absolute bottom the elevator can go from the floor
    public static double elevatorLevelMiddle = 17.0; //48.5 - 23.5; //2nd Stage height - height of intake arm
    public static double elevatorLevelTop = 35.0;  //74.7 - 23.5; //3rd Stage height - height of intake arm
    
  /* Hatch Manipulator Constants */
  /* Hatch Manipulator NOT being used */
    public static int hatchManipulatorPWMID = 4;
    public static int hatchManipulatorIntegratedCounterDIOID = 0;

    public static double internalSeatMotorGearRatio = 1 / 174.9;

    public static int hatchManipulatorLowerPositionBound = 0;
    public static int hatchManipulatorHighPositionBound = 90; ///Change

    public static double hatchManipulatorMotorSpeed = 0.5; //1/2 of the speed of the victor
  /* Intake Constants */
    public static int intakeMotorPWMID = 3;
    public static int intakeLimitSwitchDIO = 3;

    public static double intakeMotorInSpeed = -0.5; //-50%
    public static double intakeMotorOutSpeed = 0.5; //50%

    public static double intakeOutMotorTimeout = 3; //The motors will continue to spin 3 seconds after the ball leaves the net (limit switch is unpressed)

  /* Intake Arm Constants */
    public static int intakeArmPWMID = 2; //PWM Slot 2 for Spark
    public static int intakeArmPotAIO = 0; //ANALOG IO Slot 0

    public static double intakeArmMotorGearRotPerSec = 90;


    public static double intakeArmBallFeedPosition = 0; //CHANGE 
    public static double intakeArmUprightPosition = 0; //CHANGE

    public static double intakeArmAbsoluteBottom = 0; //Physical MINIMIUM that the arm can travel down
    public static double intakeArmAbsoluteTop = 90; //Physical MAXIMIUM angle that the arm can travel up

    public static double intakeArmRangeOfMotion = 300; //The degrees the Pot (420 dude) can rotate around

    public static double intakeArmAngleTolerance = 1; //1-degree tolerance for setpoint
  /* HID Constants */

    //XBOX Controller port 
    public static String driveJoystickName = "Logitech Dual Action";
    public static String auxJoystickName = "Controller (HORIPAD S)";

    //Sets constant for the sensitivity coefficent
    // ****Equation stolen from CHS in 2018****
    // Look at for more info: (GSE) (https://docs.google.com/document/d/1u_Ok_WUALfA7y753FN4YekBa_08XioyMvfyD-jt44PE/edit?usp=sharing)
    public static double sensitivityCoefficient = 0.01;

    //Sets constant for the deadband for both joysticks
    public static double joystickDeadband = 0.05; //% 
}