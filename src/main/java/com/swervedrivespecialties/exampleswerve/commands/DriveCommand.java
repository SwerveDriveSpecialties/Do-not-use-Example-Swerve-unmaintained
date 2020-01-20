package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {

        double minSS = DrivetrainSubsystem.getInstance().getMinControllerSpeed();
        double additionalSS = Robot.getOi().getAnalogSpeedScale();
        double speedScale = minSS + (1 - minSS) * additionalSS * additionalSS;

        double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
        //forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = speedScale * Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
        //strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = speedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.getOi().getPrimaryJoystick().getRawAxis(4);
        //rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = speedScale * Math.copySign(Math.pow(rotation, 2.0), rotation);

        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, DrivetrainSubsystem.getInstance().getFieldOriented());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
