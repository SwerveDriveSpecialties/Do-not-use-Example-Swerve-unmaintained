/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.ShooterTable;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RunShooterFromVision extends CommandBase {
  Shooter _shooter;
  Limelight _ll = Limelight.getInstance();
  double _curTargVelo;
  double _actuatorVal;

  public RunShooterFromVision(Shooter shooter) {
    _shooter = shooter;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTER INITIALIZING");
    getSpeed();
    _shooter.runShooter(_curTargVelo, _actuatorVal);
  }

  @Override 
  public void execute(){
    getSpeed();
    _shooter.runShooter(_curTargVelo, _actuatorVal);
    //System.out.println("hudghfhew");
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  @Override 
  public void end(boolean interrupted){
    _shooter.runShooter(0.0, 0);
  }

  private void getSpeed(){
    //if (_ll.getDistanceToTarget(Target.HIGH) > 0 && _ll.getDistanceToTarget(Target.HIGH) < 420){
      //double dist = util.inchesToFeet(_ll.getDistanceToTarget(Target.HIGH));
      double dist = 22;
      double targetSpeed = ShooterTable.getInstance().CalcShooterValues(dist).MotorTargetRPM;
      double targetActuatorVal = ShooterTable.getInstance().CalcShooterValues(dist).ActuatorVal;
      _curTargVelo = targetSpeed;
      _actuatorVal = targetActuatorVal;

    }
 // }
}
