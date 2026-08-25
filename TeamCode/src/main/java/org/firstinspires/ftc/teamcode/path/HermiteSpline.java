package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class HermiteSpline extends Movement {

    private static final int COARSE_SAMPLES_PER_SEGMENT = 20;
    private static final int REFINE_SAMPLES = 20;
    private static final double COMPLETION_PARAM_EPSILON = 0.02;

    private final Pose[] points;
    private final double[] tangentX, tangentY;
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
            } else if (i == points.length - 1) {
                tangentX[i] = points[i].x - points[i - 1].x;
                tangentY[i] = points[i].y - points[i - 1].y;
            } else {
                tangentX[i] = (points[i + 1].x - points[i - 1].x) / 2d;
                tangentY[i] = (points[i + 1].y - points[i - 1].y) / 2d;
            }
        }
    }

    private Pose evaluate(double u) {

        double clampedU = MathHelper.clamp(u, 0, numSegments);
        int segment = Math.min((int) clampedU, numSegments - 1);
        double t = clampedU - segment;

        //cubic hermite interpolation

        Pose p0 = points[segment];
        Pose p1 = points[segment + 1];

        double t2 = t * t;
        double t3 = t2 * t;

        double h00 = 2d * t3 - 3d * t2 + 1;
        double h10 = t3 - 2d * t2 + t;
        double h01 = -2d * t3 + 3d * t2;
        double h11 = t3 - t2;

        double x = h00 * p0.x + h10 * tangentX[segment] + h01 * p1.x + h11 * tangentX[segment + 1];
        double y = h00 * p0.y + h10 * tangentY[segment] + h01 * p1.y + h11 * tangentY[segment + 1];

        double heading = p0.heading + MathHelper.normalizeAngleRad(p1.heading - p0.heading) * t;

        return new Pose(x, y, heading);
    }

    private double findBestParam(Pose currentPose) {

        int totalCoarseSamples = COARSE_SAMPLES_PER_SEGMENT * numSegments;

        double bestU = 0;
        double bestDistance = Double.MAX_VALUE;

        for (int i = 0; i <= totalCoarseSamples; i++) {

            double u = (double) numSegments * i / totalCoarseSamples;
            Pose point = evaluate(u);
            double distance = Math.hypot(point.x - currentPose.x, point.y - currentPose.y);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestU = u;
            }
        }

        double coarseStep = (double) numSegments / totalCoarseSamples;
        double windowStart = Math.max(0, bestU - coarseStep);
        double windowEnd = Math.min(numSegments, bestU + coarseStep);

        for (int i = 0; i <= REFINE_SAMPLES; i++) {

            double u = windowStart + (windowEnd - windowStart) * i / REFINE_SAMPLES;
            Pose point = evaluate(u);
            double distance = Math.hypot(point.x - currentPose.x, point.y - currentPose.y);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestU = u;
            }
        }

        return bestU;
    }

    @Override
    public Pose getTarget(Pose currentPose) {
        return evaluate(findBestParam(currentPose));
    }

    @Override
    public boolean isComplete(Pose currentPose) {
        return findBestParam(currentPose) >= numSegments - COMPLETION_PARAM_EPSILON;
    }
}
