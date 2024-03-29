// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand extends Command {

  private final LiftSubsystem liftSubsystem;
  private final Supplier<Double> controls;

  public LiftCommand(LiftSubsystem liftSubsystem, Supplier<Double> controls) {
    this.liftSubsystem = liftSubsystem;
    this.controls = controls;
    addRequirements(liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controls.get();
    liftSubsystem.manuelLiftControl(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
