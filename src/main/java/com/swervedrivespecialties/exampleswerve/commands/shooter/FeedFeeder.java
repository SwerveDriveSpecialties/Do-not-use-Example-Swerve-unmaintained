/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedFeeder extends CommandBase {
  
  Shooter _shooter;

  public FeedFeeder(Shooter shooter) {
    _shooter = shooter;
  }

  int kMaxCycles = 10;
  int numCycles;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    numCycles = 0;
    _shooter.runFeeder(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    numCycles++;
    _shooter.runFeeder(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.runFeeder(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numCycles > kMaxCycles;
  }
}
