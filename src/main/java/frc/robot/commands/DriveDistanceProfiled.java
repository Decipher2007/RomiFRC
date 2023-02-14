// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiled extends ProfiledPIDCommand {
  /** Creates a new DriveDistanceProfiled. */
  public DriveDistanceProfiled(double targetDistance, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.kPDriveProfiled,
            Constants.kIDriveProfiled,
            Constants.kDDriveProfiled,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelMetersPerSecondSquared)),
        // This should return the measurement
        () -> 0,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          drivetrain.setOutputMetersPerSecond(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
