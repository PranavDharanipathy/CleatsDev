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
    private static final int ARC_LENGTH_SAMPLES = 20;

    private static final double COMPLETION_PARAM_EPSILON = 0.02;
    private static final double COMPLETION_POSITION_EPSILON = 1;
    private static final double COMPLETION_HEADING_EPSILON = Math.toRadians(2);

    /// Cross track error at which the correction fully counterbalances the tangent.
    private static final double CROSS_TRACK_SATURATION = 4;
    private static final double MAX_CROSS_TRACK_GAIN = 2;

    protected HeadingOp headingOp = HeadingOp.tangentialHeading();
    protected boolean reversed = false;

    protected double progressStart = 0;
    protected double progressSpan = 1;

    //info is cached to prevent the redundant recalculation
    private boolean hasProjectionCache;
    private double cacheX, cacheY, cacheParam, cacheDistance;

    /// @param u ranges from 0 to {@link #getMaxParam}
    /// @return the position on the curve at parameter u
    protected abstract Pose evaluate(double u);

    /// @return the upper end of the curve's parameter range (it starts at 0)
    protected abstract double getMaxParam();

    protected double tangentAngle(double u) {

        double maxParam = getMaxParam();
        double h = maxParam * 1e-4 + 1e-6;

        double u0 = Math.max(0, u - h);
        double u1 = Math.min(maxParam, u + h);

        Pose a = evaluate(u0);
        Pose b = evaluate(u1);

        return FastMath.atan2(b.y - a.y, b.x - a.x);
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

    private double arcLength(double from, double to) {

        Pose prev = evaluate(from);
        double length = 0;

        for (int i = 1; i <= ARC_LENGTH_SAMPLES; i++) {

            Pose p = evaluate(from + (to - from) * i / ARC_LENGTH_SAMPLES);
            length += Math.hypot(p.x - prev.x, p.y - prev.y);
            prev = p;
        }

        return length;
    }

    /// A path with no meaningful length is a pure rotation, so its projection can never
    /// advance and position has to be judged only by distance to the end.
    private boolean isDegenerate() {
        return arcLength(0, getMaxParam()) < COMPLETION_POSITION_EPSILON;
    }

    private boolean positionReached(Pose currentPose) {

        Pose end = evaluate(getMaxParam());
        if (Math.hypot(currentPose.x - end.x, currentPose.y - end.y) >= COMPLETION_POSITION_EPSILON) return false;

        return isDegenerate() || findBestParam(currentPose) >= getMaxParam() - COMPLETION_PARAM_EPSILON;
    }

    private Pose resolvePose(double u) {

        Pose point = evaluate(u);

        if (headingOp == null) return point; //only if 'null' is explicitly provided

        double maxParam = getMaxParam();
        double localProgress = maxParam > 0 ? MathHelper.clamp(u / maxParam, 0, 1) : 0;
        double progress = progressStart + localProgress * progressSpan;

        double heading = headingOp.heading(progress, point.x, point.y, tangentAngle(u), reversed);

        return new Pose(point.x, point.y, heading);
    }

    @Override
    public Pose getTarget(Pose currentPose) {

        if (positionReached(currentPose)) return getEndPose();

        return resolvePose(findBestParam(currentPose));
    }

    /// Drives along the path's tangent at the nearest point, blended with a correction
    /// perpendicular to it that scales with how far off the path the robot is.
    @Override
    public Pose getDriveDirection(Pose currentPose) {

        Pose end = evaluate(getMaxParam());

        if (positionReached(currentPose)) {
            return toUnitVector(end.x - currentPose.x, end.y - currentPose.y);
        }

        double u = findBestParam(currentPose);

        //past the end the projection is clamped, so the tangent no longer leads anywhere
        if (u >= getMaxParam() - COMPLETION_PARAM_EPSILON) {
            return toUnitVector(end.x - currentPose.x, end.y - currentPose.y);
        }

        Pose point = evaluate(u);
        double tangent = tangentAngle(u);

        double vx = Math.cos(tangent);
        double vy = Math.sin(tangent);

        double towardPathX = point.x - currentPose.x;
        double towardPathY = point.y - currentPose.y;
        double error = Math.hypot(towardPathX, towardPathY);

        if (error > 0) {

            double gain = MathHelper.clamp(error / CROSS_TRACK_SATURATION, 0, MAX_CROSS_TRACK_GAIN);

            vx += (towardPathX / error) * gain;
            vy += (towardPathY / error) * gain;
        }

        return toUnitVector(vx, vy);
    }

    @Override
    public double getRemainingDistance(Pose currentPose) {

        Pose end = evaluate(getMaxParam());
        double straightLineToEnd = Math.hypot(end.x - currentPose.x, end.y - currentPose.y);

        if (positionReached(currentPose)) return straightLineToEnd;

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

    /// Cross-track distance from the robot to the nearest point on the curve.
    @Override
    public double getPathError(Pose currentPose) {
        findBestParam(currentPose);
        return cacheDistance;
    }
}
