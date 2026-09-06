package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;

/// Turns to a heading the long way around
public class ReflexTurnTo extends Rotation {

    private final double targetHeading;

    /// @param targetHeading in radians
    public ReflexTurnTo(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public static ReflexTurnTo degrees(double degrees) {
        return new ReflexTurnTo(Math.toRadians(degrees));
    }

    @Override
    protected double rotationAmount(double startHeading) {
        return HeadingOp.reflex(MathHelper.normalizeAngleRad(targetHeading - startHeading));
    }
}
