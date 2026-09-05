package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/// Pose is arithmetically purposed.
/// <p>
/// Used for pose, velocity, and acceleration as well a construct
/// to represent vectors (by ignoring the heading parameter).
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

    /// @param x is in inches
    /// @param y is in inches
    public Pose(double x, double y) {
        this (x, y, 0);
    }

    public Pose add(Pose pose) {
        return new Pose(x + pose.x, y + pose.y, heading + pose.heading);
    }

    public Pose minus(Pose pose) {
        return new Pose(x - pose.x, y - pose.y, heading - pose.heading);
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

    public void normalizeHeading() {
        heading = MathHelper.normalizeAngleRad(heading);
    }

    // conversions
    public static Pose2D poseToPose2D(Pose pose) {
        return new Pose2D(INCH, pose.x, pose.y, RADIANS, pose.heading);
    }

    public static Pose pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(INCH), pose2D.getY(INCH), pose2D.getHeading(RADIANS));
    }

    public Pose copy() {
        return new Pose(x, y, heading);
    }
}