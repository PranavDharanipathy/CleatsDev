package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/// Pose is arithmetically purposed.
/// <p>
/// Used for Pose, Pose Velocity, and Pose Acceleration.
/// <p>
/// Pose is in inches.
/// <p>
/// Pose Velocity is in inches per second.
/// <p>
/// Pose Acceleration is in inches per second squared.
/// <p>
/// Pose Jerk is in inches per second cubed.
public class Pose {

    public double x, y, heading;

    /// @param x is in inches
    /// @param y is in inches
    /// @param heading is in radians
    public Pose(double x, double y, double heading) {

        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose add(Pose pose) {
        return new Pose(x + pose.x, y + pose.y, MathHelper.normalizeAngleRad(heading + pose.heading));
    }

    public Pose minus(Pose pose) {
        return new Pose(x - pose.x, y - pose.y, MathHelper.normalizeAngleRad(heading - pose.heading));
    }

    public Pose divide(Pose pose) {
        return new Pose(x / pose.x, y / pose.y, heading / pose.heading);
    }

    public Pose multiply(Pose pose) {
        return new Pose(x * pose.x, y * pose.y, heading * pose.heading);
    }

    public Pose divideBy(double value) {
        return new Pose(x / value, y / value, heading / value);
    }

    public Pose multipleBy(double value) {
        return new Pose(x * value, y * value, heading * value);
    }

    // conversions
    public static Pose2D poseToPose2D(Pose pose) {
        return new Pose2D(INCH, pose.x, pose.y, RADIANS, pose.heading);
    }

    public static Pose pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(INCH), pose2D.getY(INCH), pose2D.getHeading(RADIANS));
    }
}