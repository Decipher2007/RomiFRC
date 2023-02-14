// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kPDriveVel = 0.1;
    public static final double kIDriveVel = 0.0000000001;
    public static final double kDDriveVel = 0.00000001;
    public static final double kPTurnVel = 0.3;
    public static final double kITurnVel = 0.0;
    public static final double kDTurnVel = 0.0;
    public static final double kPDriveProfiled = 1.2;
    public static final double kIDriveProfiled = 0.0;
    public static final double kDDriveProfiled = 0.0;
    public static final double kPTurnVelProfiled = 0.05;
    public static final double kITurnVelProfiled = 0;
    public static final double kDTurnVelProfiled = 0;
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 0.5;


    // The linear inertia gain, volts
    public static final double ksVolts = 0.5;
    public static final double ksVoltsLeft = 0.50;
    public static final double ksVoltsRight = 0.44;
    
    // The linear velocity gain, volts per (meter per second)
    public static final double kvVoltSecondsPerMeter = 1.888;
    public static final double kvVoltSecondsPerMeterLeft = 1.888;
    public static final double kvVoltSecondsPerMeterRight = 1.892;
    
    // The linear acceleration gain, volts per (meter per second squared).
    public static final double kaVoltSecondsSquaredPerMeter = 0.46138;
    
    // Combined left and right volts feedforward
    public static final SimpleMotorFeedforward kFeedForward = 
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
    
    // Left and Right motors are very different, so each has its own FF.
    public static final SimpleMotorFeedforward kLeftFeedForward = 
        new SimpleMotorFeedforward(ksVoltsLeft, kvVoltSecondsPerMeterLeft, kaVoltSecondsSquaredPerMeter);
    
    public static final SimpleMotorFeedforward kRightFeedForward = 
        new SimpleMotorFeedforward(ksVoltsRight, kvVoltSecondsPerMeterRight, kaVoltSecondsSquaredPerMeter);
}
