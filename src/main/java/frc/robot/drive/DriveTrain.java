/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  private WPI_TalonSRX leftForwardTalon;
  private WPI_TalonSRX leftBackTalon;

  private WPI_TalonSRX rightForwardTalon;
  private WPI_TalonSRX rightBackTalon;

  private DifferentialDrive diffDrive;

  //private ADXRS450_Gyro robotGyro;
  private double setPoint;

  private boolean usingMMDrive = false;
  
  private double targetAngle = 0;
  private double lockedDistance = 0;

  @Override
  public void initDefaultCommand() {
    leftForwardTalon = new WPI_TalonSRX(RobotMap.leftTalonID[0]);
    leftForwardTalon.configFactoryDefault(); //Resets Talon to prevent unexpected behavor.

    leftBackTalon = new WPI_TalonSRX(RobotMap.leftTalonID[1]);
    leftBackTalon.configFactoryDefault(); //Resets Talon to prevent unexpected behavor.

    rightForwardTalon = new WPI_TalonSRX(RobotMap.rightTalonID[0]);
    rightForwardTalon.configFactoryDefault(); //Resets Talon to prevent unexpected behavor.

    rightBackTalon = new WPI_TalonSRX(RobotMap.rightTalonID[1]);
    rightBackTalon.configFactoryDefault(); //Resets Talon to prevent unexpected behavor.

    /* Set's phase direction */ 
    leftForwardTalon.setSensorPhase(false); //Set to true if sensor is naturally out of phase. Sets sensor phase for closed loop to function correctly.
    rightForwardTalon.setSensorPhase(true); //Set to true if sensor is naturally out of phase. Sets sensor phase for closed loop to function correctly.

    leftForwardTalon.setInverted(true); //Inverts left master (and slave) to drive straight.

    /* Set's brake mode */
    leftForwardTalon.setNeutralMode(NeutralMode.Brake);
    leftBackTalon.setNeutralMode(NeutralMode.Brake);
    rightForwardTalon.setNeutralMode(NeutralMode.Brake);
    rightBackTalon.setNeutralMode(NeutralMode.Brake);

    /* Set Followers & Inversions */
    leftBackTalon.set(ControlMode.Follower, leftForwardTalon.getDeviceID()); //Set left back talon to be slave to leftForwardTalon.
    leftBackTalon.setInverted(InvertType.FollowMaster); //Follows invert of master.

    rightBackTalon.set(ControlMode.Follower, rightForwardTalon.getDeviceID()); //Set left back talon to be slave to leftForwardTalon.
    rightBackTalon.setInverted(InvertType.FollowMaster); //Follows invert of master.

    /* Set Feedback Device */
    leftForwardTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDPrimary, RobotMap.talonEncoderTimeout); 
    
    /* Configs the remote Talon's sensor as a remote sensor for the right talon */
    rightForwardTalon.configRemoteFeedbackFilter(leftForwardTalon.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.PIDRemote0, RobotMap.talonEncoderTimeout);
    
    /* Setup Sum signal to be used for distance */
    rightForwardTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.talonEncoderTimeout);				// Feedback Device of Remote Talon
		rightForwardTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.talonEncoderTimeout);	// Quadrature Encoder of current Talon

    /* Setup Difference signal to be used for turning */
		rightForwardTalon.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.talonEncoderTimeout);

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    rightForwardTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PIDPrimary, RobotMap.talonEncoderTimeout);

    /* Scale Feedback by 0.5 to half the sum of Distance (There are 2 encoders, so dividing by 2 will get the average) */
		rightForwardTalon.configSelectedFeedbackCoefficient(0.5, RobotMap.PIDPrimary, RobotMap.talonEncoderTimeout);

    /* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		rightForwardTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PIDTurn, RobotMap.talonEncoderTimeout);

    /* Scale the Feedback Sensor using a coefficient */
		rightForwardTalon.configSelectedFeedbackCoefficient(1, RobotMap.PIDTurn, RobotMap.talonEncoderTimeout);

    /* Set status frame periods to ensure we don't have stale data */
		rightForwardTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.talonEncoderTimeout);
		rightForwardTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.talonEncoderTimeout);
		rightForwardTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.talonEncoderTimeout);
		rightForwardTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.talonEncoderTimeout);
		leftForwardTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.talonEncoderTimeout);

		/* Configure neutral deadband */
		rightForwardTalon.configNeutralDeadband(RobotMap.PIDNeutralDeadband, RobotMap.talonEncoderTimeout);
		leftForwardTalon.configNeutralDeadband(RobotMap.PIDNeutralDeadband, RobotMap.talonEncoderTimeout);
		
		/* Motion Magic Configurations */
		rightForwardTalon.configMotionAcceleration(RobotMap.initialCruiseAcceleration, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configMotionCruiseVelocity(RobotMap.initialCruiseVelocity, RobotMap.talonEncoderTimeout);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		leftForwardTalon.configPeakOutputForward(+1.0, RobotMap.talonEncoderTimeout);
		leftForwardTalon.configPeakOutputReverse(-1.0, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configPeakOutputForward(+1.0, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configPeakOutputReverse(-1.0, RobotMap.talonEncoderTimeout);

		/* FPID Gains for distance servo */
		rightForwardTalon.config_kP(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kP, RobotMap.talonEncoderTimeout);
		rightForwardTalon.config_kI(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kI, RobotMap.talonEncoderTimeout);
		rightForwardTalon.config_kD(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kD, RobotMap.talonEncoderTimeout);
		rightForwardTalon.config_kF(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kF, RobotMap.talonEncoderTimeout);
		rightForwardTalon.config_IntegralZone(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kIzone, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configClosedLoopPeakOutput(RobotMap.PIDDistanceSlot, RobotMap.distanceGains.kPeakOutput, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configAllowableClosedloopError(RobotMap.PIDDistanceSlot, 0, RobotMap.talonEncoderTimeout);
    
    /**
		 * Sets the time taken for PID loop
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		rightForwardTalon.configClosedLoopPeriod(0, RobotMap.PIDClosedLoopTimeMs, RobotMap.talonEncoderTimeout);
		rightForwardTalon.configClosedLoopPeriod(1, RobotMap.PIDClosedLoopTimeMs, RobotMap.talonEncoderTimeout);

    /**
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		rightForwardTalon.configAuxPIDPolarity(false, RobotMap.talonEncoderTimeout);

    zeroEncoders();

    diffDrive = new DifferentialDrive(leftForwardTalon, rightForwardTalon);
    
    //robotGyro = new ADXRS450_Gyro();
    //robotGyro.calibrate();
  }

  public void setRawPower(double speedValue, double rotationValue) {
    usingMMDrive = false;

    diffDrive.arcadeDrive(speedValue, rotationValue, true);
  }

  public void zeroEncoders() {
    leftForwardTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.talonEncoderTimeout);
		rightForwardTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.talonEncoderTimeout);
		System.out.println("All sensors are zeroed");
  }

  /*public void gyroDriveStraight(double driveValue) {
    double currentValue = robotGyro.getAngle();
    
		double difference = setPoint - currentValue;
    double motorPower = difference/180;
    
		setRawPower(driveValue+motorPower, driveValue+motorPower);
  }*/

  public double getSetPoint() {
    return setPoint;
  }

  public void setSetPoint(double newSetPoint) {
    setPoint = newSetPoint;
  }

  public void MMDrive(double forward) {
    if(usingMMDrive == false) {
      /* Determine which slot affects which PID */
			rightForwardTalon.selectProfileSlot(RobotMap.PIDDistanceSlot, RobotMap.PIDPrimary);
      rightForwardTalon.selectProfileSlot(RobotMap.PIDTurningSlot, RobotMap.PIDTurn);

      usingMMDrive = true;
    }

    /* Calculate targets from gamepad inputs */
		double target_sensorUnits = forward * RobotMap.encoderUnitsPerRotation * 6  + lockedDistance;
		double target_turn = targetAngle;
	
		/* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
		rightForwardTalon.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn); //Sets forward distance
		leftForwardTalon.follow(rightForwardTalon, FollowerType.AuxOutput1); //Sets turn
  }

  public void setMMRotationSetpoint(double rotations) { //Sets the setpoint to the current distance + the amount of rotations
    targetAngle = rightForwardTalon.getSelectedSensorPosition(1); //Difference between the two
    lockedDistance = rightForwardTalon.getSelectedSensorPosition(0) + rotations * RobotMap.encoderUnitsPerRotation; //Get current position + the amount of rotations that the motor should travel
  }

  public void distanceMMDrive(double rotations, double speed) {
    if(usingMMDrive == false) {
      /* Determine which slot affects which PID */
			rightForwardTalon.selectProfileSlot(RobotMap.PIDDistanceSlot, RobotMap.PIDPrimary);
      rightForwardTalon.selectProfileSlot(RobotMap.PIDTurningSlot, RobotMap.PIDTurn);

      setMMRotationSetpoint(rotations);

      usingMMDrive = true;
    }

    double target_sensorUnits = speed * RobotMap.encoderUnitsPerRotation * 6  + lockedDistance;
		double target_turn = targetAngle;
	
		/* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
		rightForwardTalon.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn); //Sets forward distance
		leftForwardTalon.follow(rightForwardTalon, FollowerType.AuxOutput1); //Sets turn
  }

  public double getMMTargetDistanceSetpoint() {
    return lockedDistance;
  }

  public double getMMTargetAngleSetpoint() {
    return targetAngle;
  }
}
