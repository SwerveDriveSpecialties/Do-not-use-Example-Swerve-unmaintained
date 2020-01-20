package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {
    private static OI oi;

    private static DrivetrainSubsystem drivetrain;

    private UpdateManager updateManager = new UpdateManager(
            DrivetrainSubsystem.getInstance()
    );


    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        oi = new OI();
        drivetrain = DrivetrainSubsystem.getInstance();
        Trajectories.generateAllTrajectories();
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
