// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.misc.Constants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnDegrees extends PIDCommand {
  /** Creates a new turnDegrees. */
  public turnDegrees(double targetAngleDegrees, DrivetrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DriveConstants.kTurnP, Constants.DriveConstants.kTurnI, Constants.DriveConstants.kTurnD),
        // This should return the measurement
        drive::getHeading2,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> {
          // Use the output here
          drive.turn(output);
        });
        addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.

    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    SmartDashboard.putData(getController());
    getController().setTolerance(2, 4);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
