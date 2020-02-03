/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.intake;

import com.swervedrivespecialties.exampleswerve.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunDriveThru extends CommandBase {
  Intake _intake;
  
  /**
   * Creates a new RunFeedIn.
   */
  public RunDriveThru(Intake intake) {
    _intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.runDriveThru(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.runDriveThru(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.runDriveThru(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
