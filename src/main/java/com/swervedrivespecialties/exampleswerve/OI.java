package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.ToggleFieldOriented;
import com.swervedrivespecialties.exampleswerve.commands.ToggleMinSpeed;
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
        new JoystickButton(primaryJoystick, 3).whenPressed(new ToggleMinSpeed());
        new JoystickButton(primaryJoystick, 8).whenPressed(new ToggleFieldOriented());
    }

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
}
