package org.firstinspires.ftc.teamcode.path;

public class Turn extends Rotation {

    private final double rotation;

    /// @param rotation radians (not normalized)
    public Turn(double rotation) {
        this.rotation = rotation;
    }

    public static Turn degrees(double degrees) {
        return new Turn(Math.toRadians(degrees));
    }

    @Override
    protected double rotationAmount(double startHeading) {
        return rotation;
    }
}
