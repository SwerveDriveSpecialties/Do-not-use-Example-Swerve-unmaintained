package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Utilities;


public class DriveCommand extends Command {

    DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        boolean isFieldOriented = _drive.getFieldOriented();

        double minSpeedScale = _drive.getMinSpeed();

        //All of the holonomic drive command, fwd, strafe, and rot are scaled by speedScale.
        double controllerSpeedScale = Robot.getOi().getPrimaryJoystick().getRawAxis(3);
        controllerSpeedScale = minSpeedScale + (1 - minSpeedScale) * controllerSpeedScale * controllerSpeedScale;


        double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = controllerSpeedScale * Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = controllerSpeedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.getOi().getPrimaryJoystick().getRawAxis(4);
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = controllerSpeedScale * Math.copySign(Math.pow(rotation, 2.0), rotation);

        DrivetrainSubsystem.getInstance().drive(new Vector2(forward, strafe), rotation, isFieldOriented);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
