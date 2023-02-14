// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final PIDController m_leftController =
  new PIDController(Constants.kPDriveVel, Constants.kIDriveVel, Constants.kDDriveVel);

  private final PIDController m_rightController =
  new PIDController(Constants.kPDriveVel, Constants.kIDriveVel, Constants.kDDriveVel);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDriveDifferential(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  } 

  public void tankDriveDifferential(double leftSpeed, double rightSpeed) {
    leftSpeed = applyDeadband(leftSpeed);
    rightSpeed = applyDeadband(rightSpeed);
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDriveManuel(double xSpeed, double zRotation) {
    xSpeed = applyDeadband(xSpeed);
    zRotation = applyDeadband(zRotation);
    zRotation = applyTurnScaling(xSpeed, zRotation);
    double leftMotorOutput;
    double rightMotorOutput;
    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0){
      if (zRotation >= 0.0){
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else{
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    m_rightMotor.set(rightMotorOutput);
    m_leftMotor.set(leftMotorOutput);

  }

  public double applyDeadband(double input) {
    double deadband = 0.2;
    if (Math.abs(input) > deadband) {
      return input;
    } else {
      return 0.0;
    }
  }

  public double applyTurnScaling(double speed, double rotation) {
    double scaleFactor = 0.5;

    double turnReductionPercent = speed * scaleFactor;

    double turnCoefficent = 1 - turnReductionPercent;

    return (rotation * turnCoefficent);
  }

  public void tankDrive(double leftPower, double rightPower) {
    leftPower = applyDeadband(leftPower);
    rightPower = applyDeadband(rightPower);
    m_leftMotor.set(leftPower);
    m_rightMotor.set(rightPower);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  public void setOutputMetersPerSecond(double velocity) {

    double leftFeedForward = Constants.kFeedForward.calculate(velocity);
    double rightFeedForward = Constants.kFeedForward.calculate(velocity);

    double leftPIDVolts = m_leftController.calculate(m_leftEncoder.getRate(), velocity);
    double rightPIDVolts = m_rightController.calculate(m_rightEncoder.getRate(), velocity);

    tankDriveVolts(leftFeedForward + leftPIDVolts, rightFeedForward + rightPIDVolts);

  }

  public void turn(double output) {
    double zRotation = MathUtil.clamp(output, -0.5, 5.0);
    arcadeDriveDifferential(0, zRotation);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
