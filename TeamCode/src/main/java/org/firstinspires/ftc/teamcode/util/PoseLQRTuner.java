package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;

/// Used to automatically tune {@link PoseLQRController} parameters
public class PoseLQRTuner {

    // In case you may want a different value, do not go below 3 or above 10.
    //should stay between 3 and 10 inclusive.
    private final int LOOP_ITERATIONS_PER_TIME_CONSTANT;

    private final MecanumProfile mecanumProfile;
    private final double amaxH;

    private double lastQPositionForward, lastQVelocityForward;
    private double lastQPositionStrafe, lastQVelocityStrafe;
    private double lastQPositionHeading, lastQVelocityHeading;

    public PoseLQRTuner(MotionConstraints motionConstraints, int LOOP_ITERATIONS_PER_TIME_CONSTANT) {

        this.mecanumProfile = motionConstraints.makeMecanumProfile();
        this.amaxH = motionConstraints.getAmaxH();

        this.LOOP_ITERATIONS_PER_TIME_CONSTANT = LOOP_ITERATIONS_PER_TIME_CONSTANT;
    }

    public PoseLQRTuner(MotionConstraints motionConstraints) {
        this (motionConstraints, 10);
    }

    public PoseLQRController update(double forwardError, double forwardVelocity, double strafeError, double strafeVelocity, double headingError, double angularVelocity, double dt) {

        double omegaCap = 1d / (LOOP_ITERATIONS_PER_TIME_CONSTANT * dt);

        double maxAccelForward = mecanumProfile.getMaxAcceleration(0);
        double maxAccelStrafe = mecanumProfile.getMaxAcceleration(Math.PI / 2d);

        //forward
        double omegaForward = Math.min(solveMaxOmega(Math.abs(forwardError), Math.abs(forwardVelocity), maxAccelForward), omegaCap);

        double k1F = omegaForward * omegaForward;
        double k2F = 2d * omegaForward;

        double qPositionForward = k1F * k1F;
        double qVelocityForward = k2F * k2F - 2d * k1F;

        lastQPositionForward = qPositionForward;
        lastQVelocityForward = qVelocityForward;

        //strafe
        double omegaStrafe = Math.min(solveMaxOmega(Math.abs(strafeError), Math.abs(strafeVelocity), maxAccelStrafe), omegaCap);

        double k1S = omegaStrafe * omegaStrafe;
        double k2S = 2d * omegaStrafe;

        double qPositionStrafe = k1S * k1S;
        double qVelocityStrafe = k2S * k2S - 2d * k1S;

        lastQPositionStrafe = qPositionStrafe;
        lastQVelocityStrafe = qVelocityStrafe;

        //heading
        double omegaHeading = Math.min(solveMaxOmega(Math.abs(headingError), Math.abs(angularVelocity), amaxH), omegaCap);

        double k1H = omegaHeading * omegaHeading;
        double k2H = 2d * omegaHeading;

        double qPositionHeading = k1H * k1H;
        double qVelocityHeading = k2H * k2H - 2d * k1H;

        lastQPositionHeading = qPositionHeading;
        lastQVelocityHeading = qVelocityHeading;

        return new PoseLQRController(
                qPositionForward, qVelocityForward,
                qPositionStrafe, qVelocityStrafe,
                qPositionHeading, qVelocityHeading
        );
    }

    private double solveMaxOmega(double e0, double v0, double maxAccel) {

        //provides max velocity that's within the possible range.

        if (e0 <= 0) return v0 > 0 ? maxAccel / (2d * v0) : Double.MAX_VALUE;

        return (-v0 + Math.sqrt(v0 * v0 + e0 * maxAccel)) / e0;
    }

    public double getLastQPositionForward() {
        return lastQPositionForward;
    }

    public double getLastQVelocityForward() {
        return lastQVelocityForward;
    }

    public double getLastQPositionStrafe() {
        return lastQPositionStrafe;
    }

    public double getLastQVelocityStrafe() {
        return lastQVelocityStrafe;
    }

    public double getLastQPositionHeading() {
        return lastQPositionHeading;
    }

    public double getLastQVelocityHeading() {
        return lastQVelocityHeading;
    }

}
