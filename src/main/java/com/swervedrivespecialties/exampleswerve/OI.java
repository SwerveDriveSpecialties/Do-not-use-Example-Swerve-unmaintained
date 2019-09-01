package com.swervedrivespecialties.exampleswerve;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
}
