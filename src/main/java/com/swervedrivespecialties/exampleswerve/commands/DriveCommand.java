package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        double forward = Robot.getOi().getPrimaryJoystick().getY(GenericHID.Hand.kLeft);
        // Square the forward stick
        forward = Math.copySign(forward * forward, forward);

        double strafe = Robot.getOi().getPrimaryJoystick().getX(GenericHID.Hand.kLeft);
        // Square the strafe stick
        strafe = Math.copySign(strafe * strafe, strafe);

        double rotation = Robot.getOi().getPrimaryJoystick().getX(GenericHID.Hand.kRight);
        // Square the rotation stick and scale it down by 50% to improve controlability
        rotation = Math.copySign(rotation * rotation, rotation) * 0.5;

        DrivetrainSubsystem.getInstance().holonomicDrive(new Vector2(forward, strafe), rotation);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
