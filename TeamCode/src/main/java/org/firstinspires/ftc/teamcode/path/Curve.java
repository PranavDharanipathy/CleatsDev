package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.Pose;

/// Base class for any curve type that should be followed via nearest-point projection.
/// <p>
/// Child classes only need to define where the curve actually is at a given parameter
/// value and how far that parameter goes.
public abstract class Curve extends Movement {

    private static final int COARSE_SAMPLES_PER_UNIT = 20;
    private static final int REFINE_SAMPLES = 20;
    private static final double COMPLETION_PARAM_EPSILON = 0.02;

    /// @param u ranges from 0 to getMaxParameter()
    /// @return the position and heading on the curve at parameter u
    protected abstract Pose evaluate(double u);

    /// @return the upper end of this curve's parameter range which starts at 0
    protected abstract double getMaxParam();

    protected double findBestParam(Pose currentPose) {

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

        return bestU;
    }

    @Override
    public Pose getTarget(Pose currentPose) {
        return evaluate(findBestParam(currentPose));
    }

    @Override
    public boolean isComplete(Pose currentPose) {
        return findBestParam(currentPose) >= getMaxParam() - COMPLETION_PARAM_EPSILON;
    }
}
