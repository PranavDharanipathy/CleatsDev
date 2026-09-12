package org.firstinspires.ftc.teamcode.path.optimization;

import org.apache.commons.math3.util.FastMath;

/// Something on the field the robot must not drive into, already in field coordinates.
/// <p>
/// The outline may be concave and may be made of as many points as a curve needs.
public class Obstacle {

    // Only if we had superman,

    // Doom wouldn't stand a chance.

    private final Outline outline;

    /// @param points need at least 3 points
    public Obstacle(CollisionPoint... points) {

        if (points.length < 3) throw new IllegalArgumentException("An Obstacle needs at least 3 points!");

        outline = new Outline(points);
    }

    /// Traces a shape edge by edge, mixing straight edges and curved ones.
    /// @param x where the trace starts, inches
    /// @param y where the trace starts, inches
    public static OutlineBuilder<Obstacle> startingAt(double x, double y) {
        return new OutlineBuilder<>(x, y, Obstacle::new);
    }

    public static Obstacle rectangle(double minX, double minY, double maxX, double maxY) {

        return new Obstacle(
                new CollisionPoint(minX, minY), new CollisionPoint(maxX, minY),
                new CollisionPoint(maxX, maxY), new CollisionPoint(minX, maxY));
    }

    public static Obstacle circle(double centerX, double centerY, double radius) {
        return circle(centerX, centerY, radius, 24);
    }

    /// @param points how finely the circle is traced, more points meaning a closer fit
    public static Obstacle circle(double centerX, double centerY, double radius, int points) {
        return arc(centerX, centerY, radius, 0, 2d * Math.PI, points);
    }

    /// A wedge of a circle, closed across its straight edge. A half turn of sweep makes a semicircle.
    /// @param startAngle where the curve begins, radians
    /// @param sweepAngle how far it goes around, radians
    /// @param points how closely the curve is traced
    public static Obstacle arc(double centerX, double centerY, double radius, double startAngle, double sweepAngle, int points) {

        if (points < 3) throw new IllegalArgumentException("An arc needs at least 3 points!");

        boolean whole = Math.abs(Math.abs(sweepAngle) - 2d * Math.PI) < 1e-9;

        CollisionPoint[] corners = new CollisionPoint[whole ? points : points + 1];

        for (int i = 0; i < points; i++) {

            double angle = startAngle + sweepAngle * (whole ? (double) i / points : (double) i / (points - 1));
            corners[i] = new CollisionPoint(centerX + radius * FastMath.cos(angle), centerY + radius * FastMath.sin(angle));
        }

        if (!whole) corners[points] = new CollisionPoint(centerX, centerY);

        return new Obstacle(corners);
    }

    public boolean contains(double x, double y) {
        return outline.contains(x, y);
    }

    /// @return 0 when the point is inside the obstacle, otherwise how far outside it sits
    public double clearance(double x, double y) {
        return outline.clearance(x, y);
    }

    public double getMinX() {
        return outline.minX;
    }

    public double getMinY() {
        return outline.minY;
    }

    public double getMaxX() {
        return outline.maxX;
    }

    public double getMaxY() {
        return outline.maxY;
    }

    public int getPointCount() {
        return outline.count;
    }

    public CollisionPoint[] getPoints() {
        return outline.getPoints();
    }

    public Outline getOutline() {
        return outline;
    }
}
