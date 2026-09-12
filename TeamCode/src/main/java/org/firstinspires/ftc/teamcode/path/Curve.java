package org.firstinspires.ftc.teamcode.path;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

/// Base class for any curve type that should be followed via nearest-point projection.
/// <p>
/// Child classes only need to define where the curve actually is at a given parameter
/// value and how far that parameter goes.
public abstract class Curve extends Movement {

    private static final int COARSE_SAMPLES_PER_UNIT = 20;
    private static final int REFINE_SAMPLES = 20;

    private static final double[] GAUSS_NODES = {0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640, 0.9061798459386640};
    private static final double[] GAUSS_WEIGHTS = {0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891};

    private static final double COMPLETION_PARAM_EPSILON = 0.02;
    private static final double COMPLETION_POSITION_EPSILON = 1;
    private static final double COMPLETION_HEADING_EPSILON = Math.toRadians(2);

    protected HeadingOp headingOp = HeadingOp.tangentialHeading();
    protected boolean reversed = false;

    protected double progressStart = 0;
    protected double progressSpan = 1;

    //info is cached to prevent the redundant recalculation
    private boolean hasProjectionCache;
    private double cacheX, cacheY, cacheParam, cacheDistance;

    private Boolean degenerate;
    private Double totalLength;

    /// @param u ranges from 0 to {@link #getMaxParam}
    /// @return the position on the curve at parameter u
    protected abstract Pose evaluate(double u);

    /// @return the upper end of the curve's parameter range (it starts at 0)
    protected abstract double getMaxParam();

    protected Pose derivative(double u) {

        double maxParam = getMaxParam();
        double h = maxParam * 1e-4 + 1e-6;

        double u0 = Math.max(0, u - h);
        double u1 = Math.min(maxParam, u + h);

        Pose a = evaluate(u0);
        Pose b = evaluate(u1);

        return new Pose((b.x - a.x) / (u1 - u0), (b.y - a.y) / (u1 - u0));
    }

    protected double tangentAngle(double u) {

        Pose d = derivative(u);
        return FastMath.atan2(d.y, d.x);
    }

    protected double findBestParam(Pose currentPose) {

        if (hasProjectionCache && cacheX == currentPose.x && cacheY == currentPose.y) {
            return cacheParam;
        }

        double maxParam = getMaxParam();
        int coarseSamples = Math.max(1, (int) Math.ceil(COARSE_SAMPLES_PER_UNIT * maxParam));

        double bestU = 0;
        double bestDistance = Double.MAX_VALUE;

        for (int i = 0; i <= coarseSamples; i++) {

            double u = maxParam * i / coarseSamples;
            Pose point = evaluate(u);
            double distance = Math.hypot(point.x - currentPose.x, point.y - currentPose.y);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestU = u;
            }
        }

        double coarseStep = maxParam / coarseSamples;
        double windowStart = Math.max(0, bestU - coarseStep);
        double windowEnd = Math.min(maxParam, bestU + coarseStep);

        for (int i = 0; i <= REFINE_SAMPLES; i++) {

            double u = windowStart + (windowEnd - windowStart) * i / REFINE_SAMPLES;
            Pose point = evaluate(u);
            double distance = Math.hypot(point.x - currentPose.x, point.y - currentPose.y);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestU = u;
            }
        }

        hasProjectionCache = true;
        cacheX = currentPose.x;
        cacheY = currentPose.y;
        cacheParam = bestU;
        cacheDistance = bestDistance;

        return bestU;
    }

    //Gauss-Legendre quadrature significantly improves loop times
    private double arcLength(double from, double to) {

        double length = 0;
        double a = from;

        while (a < to - 1e-12) {

            double b = Math.min(Math.floor(a) + 1, to);

            double half = (b - a) / 2d;
            double mid = (a + b) / 2d;
            double sum = 0;

            for (int i = 0; i < GAUSS_NODES.length; i++) {

                Pose d = derivative(mid + half * GAUSS_NODES[i]);
                sum += GAUSS_WEIGHTS[i] * Math.hypot(d.x, d.y);
            }

            length += sum * half;
            a = b;
        }

        return length;
    }

    private double getTotalLength() {

        if (totalLength == null) totalLength = arcLength(0, getMaxParam());

        return totalLength;
    }

    private boolean isDegenerate() { //to deal with zero-length splines with only turning

        if (degenerate == null) degenerate = getTotalLength() < COMPLETION_POSITION_EPSILON;

        return degenerate;
    }

    /// How far along the curve's own length the parameter sits, 0 to 1. Measured by
    /// arc length rather than by parameter so unevenly spaced points don't skew it.
    protected double getLocalProgress(double u) {

        double total = getTotalLength();

        if (total <= 0) return 0;

        double maxParam = getMaxParam();

        return MathHelper.clamp(1 - arcLength(MathHelper.clamp(u, 0, maxParam), maxParam) / total, 0, 1);
    }

    private boolean positionReached(Pose currentPose) {

        Pose end = evaluate(getMaxParam());
        if (Math.hypot(currentPose.x - end.x, currentPose.y - end.y) >= COMPLETION_POSITION_EPSILON) return false;

        return isDegenerate() || findBestParam(currentPose) >= getMaxParam() - COMPLETION_PARAM_EPSILON;
    }

    private boolean isProjectionUseful(Pose currentPose) {

        if (isDegenerate() || positionReached(currentPose)) return false;

        return findBestParam(currentPose) < getMaxParam() - COMPLETION_PARAM_EPSILON;
    }

    private Pose resolvePose(double u) {

        Pose point = evaluate(u);

        if (headingOp == null) return point; //only if 'null' is explicitly provided

        double progress = progressStart + getLocalProgress(u) * progressSpan;

        double heading = headingOp.heading(progress, point.x, point.y, tangentAngle(u), reversed);

        return new Pose(point.x, point.y, heading);
    }

    @Override
    public Pose getTarget(Pose currentPose) {

        if (isDegenerate() || positionReached(currentPose)) return getEndPose();

        return resolvePose(findBestParam(currentPose));
    }

    @Override
    public Pose getTangentDirection(Pose currentPose) {

        if (!isProjectionUseful(currentPose)) {

            Pose end = evaluate(getMaxParam());
            return toUnitVector(end.x - currentPose.x, end.y - currentPose.y);
        }

        double tangent = tangentAngle(findBestParam(currentPose));

        return new Pose(Math.cos(tangent), Math.sin(tangent));
    }

    @Override
    public double getSignedCrossTrack(Pose currentPose) {

        if (!isProjectionUseful(currentPose)) return 0;

        double u = findBestParam(currentPose);
        Pose point = evaluate(u);
        double tangent = tangentAngle(u);

        double towardPathX = point.x - currentPose.x;
        double towardPathY = point.y - currentPose.y;

        return -towardPathX * Math.sin(tangent) + towardPathY * Math.cos(tangent);
    }

    @Override
    public double getCurvature(Pose currentPose, double aheadDistance, double allowedCut) {

        if (!isProjectionUseful(currentPose) || allowedCut <= 0) return 0;

        final double maxParam = getMaxParam();

        double u = findBestParam(currentPose);
        Pose here = derivative(u);

        double perParam = Math.hypot(here.x, here.y);

        if (perParam <= 0) return 0;

        double at = MathHelper.clamp(u + aheadDistance / perParam, 0, maxParam);
        double bend = 0;

        //a bend isn't worth slowing for if the robot can SAFELY cut across it
        for (double window = allowedCut; window <= allowedCut * 64; window *= 2) {

            double half = window / (2d * perParam);

            double from = MathHelper.clamp(at - half, 0, maxParam), to = MathHelper.clamp(at + half, 0, maxParam);

            if (to <= from) continue;

            Pose leaving = derivative(from), arriving = derivative(to);

            double span = (Math.hypot(leaving.x, leaving.y) + Math.hypot(arriving.x, arriving.y)) / 2d * (to - from);

            if (span <= 0) continue;

            double turn = Math.abs(MathHelper.normalizeAngleRad(
                    FastMath.atan2(arriving.y, arriving.x) - FastMath.atan2(leaving.y, leaving.x)));

            if (span * turn / 8d <= allowedCut) continue;

            bend = Math.max(bend, turn / span);
        }

        return bend;
    }

    @Override
    public double getRemainingDistance(Pose currentPose) {

        Pose end = evaluate(getMaxParam());
        double straightLineToEnd = Math.hypot(end.x - currentPose.x, end.y - currentPose.y);

        if (isDegenerate() || positionReached(currentPose)) return straightLineToEnd;

        return Math.max(arcLength(findBestParam(currentPose), getMaxParam()), straightLineToEnd);
    }

    @Override
    public boolean isComplete(Pose currentPose) {

        if (!positionReached(currentPose)) return false;

        double headingError = MathHelper.normalizeAngleRad(currentPose.heading - getEndPose().heading);
        return Math.abs(headingError) < COMPLETION_HEADING_EPSILON;
    }

    @Override
    public Pose getEndPose() {
        return resolvePose(getMaxParam());
    }

    /// Reads curve without needing robot pose.
    /// @param t 0 to 1 across the curve's parameter range
    public Pose sample(double t) {
        return resolvePose(MathHelper.clamp(t, 0, 1) * getMaxParam());
    }

    /// Cross-track distance from the robot to the nearest point on the curve.
    @Override
    public double getPathError(Pose currentPose) {
        findBestParam(currentPose);
        return cacheDistance;
    }
}
