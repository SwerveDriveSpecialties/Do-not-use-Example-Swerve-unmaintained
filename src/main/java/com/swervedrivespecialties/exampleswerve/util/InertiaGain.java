package com.swervedrivespecialties.exampleswerve.util;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;

/**
 * Add your docs here.
 */
public class InertiaGain {
    double kForwardMult = 1;
    double kStrafeMult = 1;
    double kRotationMult = 1;

    public InertiaGain(double iForward, double iStrafe, double iRotation){
        kForwardMult = 1 + iForward;
        kStrafeMult = 1 + iStrafe;
        kRotationMult = 1 + iRotation;
    }

    public static InertiaGain id = new InertiaGain(0, 0, 0);

    public HolonomicDriveSignal apply(HolonomicDriveSignal rawSignal){
        Vector2 vec = rawSignal.getTranslation();
        double x = vec.x;
        double y = vec.y;
        double phi = rawSignal.getRotation();
        x *= kForwardMult;
        y *= kStrafeMult;
        phi *= kRotationMult;
        return new HolonomicDriveSignal(new Vector2(x, y), phi, rawSignal.isFieldOriented());
    }
}