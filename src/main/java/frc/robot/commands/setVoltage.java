// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class setVoltage extends CommandBase {
  private double voltage;
  private DrivetrainSubsystem drivetrain;

  /** Creates a new setVoltage. */
public setVoltage(DrivetrainSubsystem drivetrain) {

  this.drivetrain = drivetrain;
  addRequirements(drivetrain);
  SmartDashboard.putNumber("volt", 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltage = SmartDashboard.getNumber("volt", 0.1);
    drivetrain.tankDriveVolts(voltage, voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.tankDriveVolts(0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
