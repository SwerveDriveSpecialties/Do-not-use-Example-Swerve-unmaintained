package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotContainer;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends CommandBase {

    DrivetrainSubsystem _drive;

    public DriveCommand(DrivetrainSubsystem subsystem) {
        _drive = subsystem;
        addRequirements(_drive);
    }

    @Override
    public void execute() {
        double minSS = DrivetrainSubsystem.getInstance().getMinControllerSpeed();
        double additionalSS =  Robot.getRobotContainer().getPrimaryRightTrigger();
        double speedScale = minSS + (1 - minSS) * additionalSS * additionalSS;

        double forward = -Robot.getRobotContainer().getPrimaryLeftYAxis();
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = speedScale * Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.getRobotContainer().getPrimaryLeftXAxis();
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = speedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.getRobotContainer().getPrimaryRightXAxis();
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = speedScale * Math.copySign(Math.pow(rotation, 2.0), rotation);

        _drive.drive(new Translation2d(forward, strafe), rotation, _drive.getFieldOriented());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
