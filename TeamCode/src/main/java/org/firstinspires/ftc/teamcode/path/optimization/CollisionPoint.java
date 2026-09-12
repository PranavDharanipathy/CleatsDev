package org.firstinspires.ftc.teamcode.path.optimization;

/// A corner of the robot's outline, measured from the robot's center in inches.
public class CollisionPoint {

    public final double x, y;

    /// @param x how far to the robot's right, inches
    /// @param y how far toward the robot's front, inches
    public CollisionPoint(double x, double y) {

        this.x = x;
        this.y = y;
    }
}
