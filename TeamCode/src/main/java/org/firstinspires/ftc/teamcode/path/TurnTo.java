package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;

/// Turns to a heading the short way around
public class TurnTo extends Rotation {

    private final double targetHeading;

    /// @param targetHeading in radians
    public TurnTo(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public static TurnTo degrees(double degrees) {
        return new TurnTo(Math.toRadians(degrees));
    }

    @Override
    protected double rotationAmount(double startHeading) {
        return MathHelper.normalizeAngleRad(targetHeading - startHeading);
    }
}
