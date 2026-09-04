package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class HermiteSpline extends Curve {

    private final Pose[] points;
    private final double[] tangentX, tangentY;
    private final double[] accelX, accelY;
    private final int numSegments;

    public HermiteSpline(Pose... points) {

        if (points.length < 2) throw new IllegalArgumentException("HermiteSpline needs at least 2 points!");

        this.points = points;
        this.numSegments = points.length - 1;

        tangentX = new double[points.length];
        tangentY = new double[points.length];

        for (int i = 0; i < points.length; i++) {

            if (i == 0) {
                tangentX[i] = points[1].x - points[0].x;
                tangentY[i] = points[1].y - points[0].y;
            }
            else if (i == points.length - 1) {
                tangentX[i] = points[i].x - points[i - 1].x;
                tangentY[i] = points[i].y - points[i - 1].y;
            }
            else {
                tangentX[i] = (points[i + 1].x - points[i - 1].x) / 2d;
                tangentY[i] = (points[i + 1].y - points[i - 1].y) / 2d;
            }
        }

        //we want 0 curvature at the endpoints, and estimate 2nd difference
        // (2nd difference is how fast the path's direction is changing
        // given the spline parameter (u)) at each interior point from the
        // neighboring positions.
        accelX = new double[points.length];
        accelY = new double[points.length];

        for (int i = 1; i < points.length - 1; i++) {
            accelX[i] = points[i + 1].x - 2d * points[i].x + points[i - 1].x;
            accelY[i] = points[i + 1].y - 2d * points[i].y + points[i - 1].y;
        }
    }

    /// Enables replanning for this spline. If the robot strays farther than
    /// offShootDistance from the path, the spline is replanned from the robot's
    /// current pose through the remaining waypoints and will end at the same
    /// end pose.
    public HermiteSpline setReplan(double offShootDistance) {
        setReplanner(this::replanFrom, offShootDistance);
        return this;
    }

    private HermiteSpline replanFrom(Pose currentPose) {

        //replanning algorithm

        double u = findBestParam(currentPose);

        //first waypoint ahead of the current projection
        int firstAhead = MathHelper.clamp((int) Math.floor(u) + 1, 1, numSegments);

        final int aheadCount = numSegments - firstAhead + 1;

        Pose[] newPoints = new Pose[1 + aheadCount];
        newPoints[0] = currentPose;
        for (int k = 0; k < aheadCount; k++) {
            newPoints[1 + k] = points[firstAhead + k];
        }

        return new HermiteSpline(newPoints).setReplan(getReplanOffShootDistance());
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

        double x = h00 * p0.x + h10 * tangentX[segment] + h20 * accelX[segment] + h01 * p1.x + h11 * tangentX[segment + 1] + h21 * accelX[segment + 1];
        double y = h00 * p0.y + h10 * tangentY[segment] + h20 * accelY[segment] + h01 * p1.y + h11 * tangentY[segment + 1] + h21 * accelY[segment + 1];

        double heading = p0.heading + MathHelper.normalizeAngleRad(p1.heading - p0.heading) * t;

        return new Pose(x, y, heading);
    }

    @Override
    protected double getMaxParam() {
        return numSegments;
    }
}
