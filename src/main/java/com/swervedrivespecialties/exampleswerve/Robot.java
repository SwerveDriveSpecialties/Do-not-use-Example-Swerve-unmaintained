package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot{

    static RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Trajectories.generateAllTrajectories();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (!isDisabled()){
            robotContainer.logAllData();
        }
    }

    @Override
    public void autonomousInit() {
        robotContainer.setupLogging(true);
    }

    @Override
    public void autonomousPeriodic() {
        DrivetrainSubsystem.getInstance().drive(new Translation2d(0., 0), 0, true);
    }

    @Override
    public void teleopInit() {
        robotContainer.setupLogging(false);
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
