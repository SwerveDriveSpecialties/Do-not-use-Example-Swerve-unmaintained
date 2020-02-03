/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.intake;

import com.swervedrivespecialties.exampleswerve.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Add your docs here.
 */
public class IntakeSubsystemCommands {
    public static Intake intake = Intake.getInstance();

    public static CommandBase getRunFeedInCommand(){
        return new RunFeedIn(intake);
    }

    public static CommandBase getRunIndexCommand(){
        return new RunIndex(intake);
    }

    public static CommandBase getRunDriveThruCommand(){
        return new RunDriveThru(intake);
    }
}
