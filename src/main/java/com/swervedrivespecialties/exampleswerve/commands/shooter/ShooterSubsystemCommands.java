/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.shooter;

import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSubsystemCommands {
    public static Shooter shooter = Shooter.getInstance();

    public static CommandBase getRunShooterCommand(double speed){
        return new RunShooter(shooter, speed);
    }

    public static CommandBase getRunShooterCommand(){
        return new RunShooter(shooter);
    }

    public static CommandBase getFeedFeederCommand(){
        return new FeedFeeder(shooter);
    }
}
