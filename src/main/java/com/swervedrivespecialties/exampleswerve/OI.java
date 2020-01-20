package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.FollowTrajectory;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);

    public OI() {
        // Back button zeroes the drivetrain
        new JoystickButton(primaryJoystick, 7).whenPressed(
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroscope())
        );

        new JoystickButton(primaryJoystick, 8).whenPressed(
            new InstantCommand(() -> DrivetrainSubsystem.getInstance().toggleFieldOriented())
        );

        new JoystickButton(primaryJoystick, 4).whenPressed(
            new InstantCommand(() -> DrivetrainSubsystem.getInstance().toggleMinControllerSpeed())
        );

        new JoystickButton(primaryJoystick, 6).whenPressed(new FollowTrajectory(Trajectories.testTrajectorySupplier));
    }

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }

    public double getAnalogSpeedScale(){
        return primaryJoystick.getRawAxis(3);
    }
}
