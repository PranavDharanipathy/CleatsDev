package org.firstinspires.ftc.teamcode.following.chassis;

import org.firstinspires.ftc.teamcode.util.BinarySearch;
import org.firstinspires.ftc.teamcode.util.MathHelper;

public class BrakingModel {

    private final double[] speeds;
    private final double[] forwardDistances;
    private final double[] strafeDistances;
    private final double[] shapes;

    private final double[] angularSpeeds;
    private final double[] angularDistances;

    private final double latency;
    private final double margin;
    private final double angularMargin;

    /// @param speeds ascending test speeds in inches per second, for all three directions
    /// @param forwardDistances stopping distances recorded at each speed while driving forward
    /// @param strafeDistances stopping distances recorded at each speed while strafing
    /// @param diagonalDistances stopping distances recorded at each speed while moving diagonally
    /// @param angularSpeeds ascending test speeds in radians per second
    /// @param angularDistances stopping angles recorded at each angular speed
    /// @param latency extra pose-estimate delay, in seconds, beyond what is already accounted for in the tables
    /// @param margin amount of slack, in inches, used to determine when an axis is considered settled
    /// @param angularMargin same slack as margin, but measured in radians
    public BrakingModel(
            double[] speeds,
            double[] forwardDistances,
            double[] strafeDistances,
            double[] diagonalDistances,
            double[] angularSpeeds,
            double[] angularDistances,
            double latency,
            double margin,
            double angularMargin
    ) {

        if (speeds.length < 2 || angularSpeeds.length < 2) throw new IllegalArgumentException("BrakingModel needs at least 2 samples per table!");

        if (forwardDistances.length != speeds.length
                || strafeDistances.length != speeds.length
                || diagonalDistances.length != speeds.length
                || angularDistances.length != angularSpeeds.length) {
            throw new IllegalArgumentException("BrakingModel table lengths must match their speed axis!");
        }

        this.speeds = speeds;
        this.forwardDistances = forwardDistances;
        this.strafeDistances = strafeDistances;

        this.angularSpeeds = angularSpeeds;
        this.angularDistances = angularDistances;

        this.latency = latency;
        this.margin = margin;
        this.angularMargin = angularMargin;

        shapes = new double[speeds.length];

        for (int i = 0; i < speeds.length; i++) {
            shapes[i] = MecanumProfile.solveShapeExponent(forwardDistances[i], strafeDistances[i], diagonalDistances[i]);
        }
    }

    public double getStoppingDistance(double angle, double speed) {

        speed = Math.abs(speed);

        if (speed <= 0) return 0;

        return interpolateDirectional(angle, speed) + speed * latency;
    }

    public double getAngularStoppingDistance(double angularSpeed) {

        angularSpeed = Math.abs(angularSpeed);

        if (angularSpeed <= 0) return 0;

        return interpolate(angularSpeeds, angularDistances, angularSpeed) + angularSpeed * latency;
    }

    private double interpolateDirectional(double angle, double speed) {

        if (speed <= speeds[0]) return lameValueAt(0, angle) * speed / speeds[0];

        if (speed > speeds[speeds.length - 1]) {

            int last = speeds.length - 1;

            double previous = lameValueAt(last - 1, angle);
            double end = lameValueAt(last, angle);

            return end + (end - previous) * (speed - speeds[last]) / (speeds[last] - speeds[last - 1]);
        }

        //efficiency, ends up being O(log n) instead of O(n)
        int i = BinarySearch.INSTANCE.firstGreaterOrEqual(speeds, speed);

        double low = lameValueAt(i - 1, angle);
        double high = lameValueAt(i, angle);

        return low + (high - low) * (speed - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
    }

    private double lameValueAt(int index, double angle) {
        return MathHelper.lameValue(angle, forwardDistances[index], strafeDistances[index], shapes[index]);
    }

    private static double interpolate(double[] xs, double[] ys, double x) {

        if (x <= xs[0]) return ys[0] * x / xs[0];

        if (x > xs[xs.length - 1]) {

            int last = xs.length - 1;

            return ys[last] + (ys[last] - ys[last - 1]) * (x - xs[last]) / (xs[last] - xs[last - 1]);
        }

        //efficiency, ends up being O(log n) instead of O(n)
        int i = BinarySearch.INSTANCE.firstGreaterOrEqual(xs, x);

        return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    }

    public double getLatency() {
        return latency;
    }

    public double getMargin() {
        return margin;
    }

    public double getAngularMargin() {
        return angularMargin;
    }
}
