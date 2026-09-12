package org.firstinspires.ftc.teamcode.path.optimization;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.util.Pose;

public class RobotFrame {

    private final Outline outline;

    //the outline as forward and left offsets (required for it to be transformed to field-relative)
    private final double[] forward, left;

    private final double radius, innerRadius;

    /// @param points at least 3 corners of the robot, in order around it or in any order if it is convex
    public RobotFrame(CollisionPoint... points) {

        if (points.length < 3) throw new IllegalArgumentException("A RobotFrame needs at least 3 points!");

        outline = new Outline(points);

        forward = new double[outline.count];
        left = new double[outline.count];

        for (int i = 0; i < outline.count; i++) {

            //this part is in cartesian system
            forward[i] = outline.y[i];
            left[i] = -outline.x[i];
        }

        radius = outline.radiusFrom(0, 0);
        innerRadius = outline.contains(0, 0) ? outline.distanceToEdge(0, 0) : 0;
    }

    /// Traces the robot edge by edge, mixing straight edges and curved ones.
    /// @param x how far to the robot's right the trace starts, inches
    /// @param y how far toward the robot's front the trace starts, inches
    public static OutlineBuilder<RobotFrame> startingAt(double x, double y) {
        return new OutlineBuilder<>(x, y, RobotFrame::new);
    }

    /// @param robotPose where the robot is on the field
    /// @return every corner at its field position, ordered around the outline
    public Pose[] getGlobalPose(Pose robotPose) {

        final double cos = FastMath.cos(robotPose.heading);
        final double sin = FastMath.sin(robotPose.heading);

        Pose[] global = new Pose[outline.count];

        for (int i = 0; i < outline.count; i++) {

            global[i] = new Pose(
                    robotPose.x + forward[i] * cos - left[i] * sin,
                    robotPose.y + forward[i] * sin + left[i] * cos
            );
        }

        return global;
    }

    /// @return how far the furthest corner sits from the robot's center
    public double getRadius() {
        return radius;
    }

    /// @return the biggest circle centered on the robot that still fits inside it
    public double getInnerRadius() {
        return innerRadius;
    }

    public int getPointCount() {
        return outline.count;
    }

    /// @return the robot's own outline, as it was given, measured from its center
    public Outline getOutline() {
        return outline;
    }

    /// @return the outline's corners, wound counter-clockwise, measured from the robot's center
    public CollisionPoint[] getPoints() {
        return outline.getPoints();
    }

    //the same transform without allocating, for the sweep checks
    protected void writeGlobal(double px, double py, double heading, double[] outX, double[] outY) {

        final double cos = FastMath.cos(heading);
        final double sin = FastMath.sin(heading);

        for (int i = 0; i < outline.count; i++) {
            outX[i] = px + forward[i] * cos - left[i] * sin;
            outY[i] = py + forward[i] * sin + left[i] * cos;
        }
    }
}
