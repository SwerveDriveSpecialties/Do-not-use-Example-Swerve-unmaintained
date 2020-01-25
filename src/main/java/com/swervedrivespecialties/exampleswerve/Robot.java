package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import edu.wpi.first.wpilibj.TimedRobot;
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
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
