package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class Rotation extends Movement {

    private static final double COMPLETION_HEADING_EPSILON = Math.toRadians(2);

    //MathHelper.normalizeAngleRad only preserves values inside this, so what is reported stops here
    private static final double MAX_REPORTED_ERROR = Math.PI * 0.99;

    private Pose startPose;
    private double targetRotation;
    private double rotated;
    private double lastHeading;
    private boolean started;

    /// @return the signed amount to turn in radians, not normalized
    protected abstract double rotationAmount(double startHeading);

    private void track(Pose currentPose) {

        if (!started) {

            startPose = new Pose(currentPose.x, currentPose.y, currentPose.heading);
            lastHeading = currentPose.heading;
            targetRotation = rotationAmount(currentPose.heading);
            rotated = 0;
            started = true;

            return;
        }

        if (currentPose.heading == lastHeading) return;

        rotated += MathHelper.normalizeAngleRad(currentPose.heading - lastHeading);
        lastHeading = currentPose.heading;
    }

    public double getTargetRotation() {
        return targetRotation;
    }

    public double getRotated() {
        return rotated;
    }

    public double getRemainingRotation() {
        return targetRotation - rotated;
    }

    public boolean hasStarted() {
        return started;
    }

    @Override
    public void reset() {
        started = false;
    }

    @Override
    public Pose getTarget(Pose currentPose) {

        track(currentPose);

        double step = MathHelper.clamp(getRemainingRotation(), -MAX_REPORTED_ERROR, MAX_REPORTED_ERROR);

        return new Pose(startPose.x, startPose.y, currentPose.heading + step);
    }

    @Override
    public boolean isComplete(Pose currentPose) {

        track(currentPose);

        return Math.abs(getRemainingRotation()) < COMPLETION_HEADING_EPSILON;
    }

    @Override
    public Pose getEndPose() {

        if (!started) return new Pose();

        return new Pose(startPose.x, startPose.y, MathHelper.normalizeAngleRad(startPose.heading + targetRotation));
    }

    @Override
    public Pose getTangentDirection(Pose currentPose) {
        return new Pose();
    }

    @Override
    public double getRemainingDistance(Pose currentPose) {
        return 0;
    }
}
