/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooter extends CommandBase {

  private static final double kDefaultShooterSpeed = .5;

  Shooter _shooter;
  double spd;

  public RunShooter(Shooter shooter, double speed) {
    _shooter = shooter;
    spd = speed;
    addRequirements(_shooter);
  }

  public RunShooter(Shooter shooter){
    this(shooter, kDefaultShooterSpeed);
  }

  @Override
  public void initialize() {
    _shooter.runShooter(spd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.runShooter(spd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.runShooter(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
