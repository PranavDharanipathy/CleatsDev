package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class HermiteSpline extends Curve {

    private final Pose[] points;
    private final double[] directionX, directionY;
    private final double[] bendX, bendY;
    private final double[] chord;
    private final int numSegments;

    public HermiteSpline(Pose... points) {

        if (points.length < 2) throw new IllegalArgumentException("HermiteSpline needs at least 2 points!");

        this.points = points;
        this.numSegments = points.length - 1;

        chord = new double[numSegments];

        for (int i = 0; i < numSegments; i++)
            chord[i] = Math.max(Math.hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y), 1e-9);

        directionX = new double[points.length];
        directionY = new double[points.length];

        for (int i = 0; i < points.length; i++) {

            double x, y;

            if (i == 0) {
                x = points[1].x - points[0].x;
                y = points[1].y - points[0].y;
            }
            else if (i == points.length - 1) {
                x = points[i].x - points[i - 1].x;
                y = points[i].y - points[i - 1].y;
            }
            else {
                x = (points[i + 1].x - points[i - 1].x) / 2d;
                y = (points[i + 1].y - points[i - 1].y) / 2d;
            }

            double length = Math.hypot(x, y);

            directionX[i] = length > 0 ? x / length : 0;
            directionY[i] = length > 0 ? y / length : 0;
        }

        //we want 0 curvature at the endpoints, and estimate 2nd difference
        // (2nd difference is how fast the path's direction is changing
        // given the spline parameter (u)) at each interior point from the
        // neighboring positions.
        bendX = new double[points.length];
        bendY = new double[points.length];

        for (int i = 1; i < points.length - 1; i++) {

            double before = chord[i - 1], after = chord[i];

            //measured per inch rather than per point, which is the same number when the points
            // are evenly spaced and the honest one when they are not
            bendX[i] = 2d * (points[i - 1].x / (before * (before + after)) - points[i].x / (before * after) + points[i + 1].x / (after * (before + after)));
            bendY[i] = 2d * (points[i - 1].y / (before * (before + after)) - points[i].y / (before * after) + points[i + 1].y / (after * (before + after)));
        }
    }

    /// Faces the opposite direction while following
    public HermiteSpline setReversed(boolean reversed) {
        this.reversed = reversed;
        return this;
    }

    /// Sets how heading will be handles (defaults to tangential heading).
    public HermiteSpline setHeadingOp(HeadingOp headingOp) {
        this.headingOp = headingOp;
        return this;
    }

    /// Sets a different heading control for each part of the path.
    public HermiteSpline setHeadingOp(HeadingOp.Slice... slices) {
        this.headingOp = HeadingOp.compound(slices);
        return this;
    }

    public HermiteSpline setReplan(double offShootDistance) {
        setReplanner(this::replanFrom, offShootDistance);
        return this;
    }

    private HermiteSpline replanFrom(Pose currentPose) {

        double u = findBestParam(currentPose);

        //first waypoint ahead of the current projection
        int firstAhead = MathHelper.clamp((int) Math.floor(u) + 1, 1, numSegments);
        int aheadCount = numSegments - firstAhead + 1;

        Pose[] newPoints = new Pose[1 + aheadCount];
        newPoints[0] = new Pose(currentPose.x, currentPose.y, currentPose.heading);
        for (int i = 0; i < aheadCount; i++) {
            newPoints[1 + i] = points[firstAhead + i];
        }

        HermiteSpline replanned = new HermiteSpline(newPoints);

        replanned.headingOp = this.headingOp;
        replanned.reversed = this.reversed;

        double originalProgressAtReplan = progressStart + getLocalProgress(u) * progressSpan;

        replanned.progressStart = originalProgressAtReplan;
        replanned.progressSpan = (progressStart + progressSpan) - originalProgressAtReplan;

        return replanned.setReplan(getReplanOffShootDistance());
    }

    @Override
    protected Pose evaluate(double u) {

        // yo chillax bro, it's okay you ain't the G.O.A.T., I am.

        double clampedU = MathHelper.clamp(u, 0, numSegments);
        int segment = Math.min((int) clampedU, numSegments - 1);
        double t = clampedU - segment;

        //quintic hermite interpolation

        Pose p0 = points[segment];
        Pose p1 = points[segment + 1];

        double span = chord[segment];
        double square = span * span;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        double h00 /*h0*/ = -6d * t5 + 15d * t4 - 10d * t3 + 1d;
        double h10 /*h1*/ = -3d * t5 + 8d * t4 - 6d * t3 + t;
        double h20 /*h2*/ = -0.5d * t5 + 1.5d * t4 - 1.5d * t3 + 0.5d * t2;
        double h01 /*h3*/ = 6d * t5 - 15d * t4 + 10d * t3;
        double h11 /*h4*/ = -3d * t5 + 7d * t4 - 4d * t3;
        double h21 /*h5*/ = 0.5d * t5 - t4 + 0.5d * t3;

        double x = h00 * p0.x + h10 * directionX[segment] * span + h20 * bendX[segment] * square
                + h01 * p1.x + h11 * directionX[segment + 1] * span + h21 * bendX[segment + 1] * square;
        double y = h00 * p0.y + h10 * directionY[segment] * span + h20 * bendY[segment] * square
                + h01 * p1.y + h11 * directionY[segment + 1] * span + h21 * bendY[segment + 1] * square;

        double heading = p0.heading + MathHelper.normalizeAngleRad(p1.heading - p0.heading) * t;

        return new Pose(x, y, heading);
    }

    @Override
    protected Pose derivative(double u) {

        double clampedU = MathHelper.clamp(u, 0, numSegments);
        int segment = Math.min((int) clampedU, numSegments - 1);
        double t = clampedU - segment;

        Pose p0 = points[segment];
        Pose p1 = points[segment + 1];

        double span = chord[segment];
        double square = span * span;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;

        double d00 /*d0*/ = -30d * t4 + 60d * t3 - 30d * t2;
        double d10 /*d1*/ = -15d * t4 + 32d * t3 - 18d * t2 + 1d;
        double d20 /*d2*/ = -2.5d * t4 + 6d * t3 - 4.5d * t2 + t;
        double d01 /*d3*/ = 30d * t4 - 60d * t3 + 30d * t2;
        double d11 /*d4*/ = -15d * t4 + 28d * t3 - 12d * t2;
        double d21 /*d5*/ = 2.5d * t4 - 4d * t3 + 1.5d * t2;

        double x = d00 * p0.x + d10 * directionX[segment] * span + d20 * bendX[segment] * square
                + d01 * p1.x + d11 * directionX[segment + 1] * span + d21 * bendX[segment + 1] * square;
        double y = d00 * p0.y + d10 * directionY[segment] * span + d20 * bendY[segment] * square
                + d01 * p1.y + d11 * directionY[segment + 1] * span + d21 * bendY[segment + 1] * square;

        return new Pose(x, y);
    }

    @Override
    protected double getMaxParam() {
        return numSegments;
    }
}
