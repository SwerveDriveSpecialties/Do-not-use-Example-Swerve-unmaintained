/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

public class runInfeed extends CommandBase {
  
  Infeed _infeed = Infeed.get_instance();

  public runInfeed(Infeed infeed) {
    _infeed = infeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _infeed.runInfeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _infeed.runInfeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _infeed.stopInfeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
