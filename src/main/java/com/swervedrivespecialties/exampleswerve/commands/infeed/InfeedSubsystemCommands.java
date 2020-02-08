/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Add your docs here.
 */
public class InfeedSubsystemCommands {
    public static Infeed infeed = Infeed.get_instance();

    
    public static CommandBase getConveyToShootCommand(){
        return new conveyToShoot(infeed);
    }

    public static CommandBase getConveyCommand(){
        return new convey(infeed);
    }  

    public static CommandBase getRunInfeedCommand(){
        return new runInfeed(infeed);
    }

    public static CommandBase getRunSingulatorCommand(){
        return new runSingulator(infeed);
    }
}
