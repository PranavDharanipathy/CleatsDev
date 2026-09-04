package org.firstinspires.ftc.teamcode.following;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.path.Movement;

public class PathController {

    // Wait,
    // Run that back,

    // Nothing ever goes wrong in Cleats.

    private enum Mode {
        TRANSIT, PRECISION
    }

    private final Chassis chassis;
    private final MotionConstraints motionConstraints;
    private final MecanumProfile mecanumProfile;

    private final FinalLocalizer localizer;
    private Pose pose, velocity, acceleration, jerk;

    private double dt;

    private final PoseLQRController poseLQR;
    private final PrecisionModeThresholds precisionModeThresholds;

    private Movement currentMovement;
    private boolean precisionStopEnabled;
    private Mode mode;

    public PathController(Chassis chassis, FinalLocalizer localizer, MotionConstraints motionConstraints, PoseLQRController poseLQR, PrecisionModeThresholds precisionModeThresholds) {

        this.chassis = chassis;

        this.localizer = localizer;

        this.motionConstraints = motionConstraints;
        mecanumProfile = this.motionConstraints.makeMecanumProfile();

        this.poseLQR = poseLQR;
        this.precisionModeThresholds = precisionModeThresholds;

        mode = Mode.TRANSIT;
    }

    /// Call once to start following a path.
    public void follow(Movement movement) {
        follow(movement, true);
    }

    /// Call once to start following a path.
    /// @param precisionStop whether to use precision mode for end pose correction or not
    public void follow(Movement movement, boolean precisionStop) {

        currentMovement = movement;
        precisionStopEnabled = precisionStop;
        mode = Mode.TRANSIT;
    }

    /// Must be called every loop.
    public void update() {

        localizer.update();

        dt = localizer.getDeltaTime();

        pose = localizer.getPose();
        velocity = localizer.getVelocity();
        acceleration = localizer.getAcceleration();
        jerk = localizer.getJerk();

        if (currentMovement == null) return;

        if (!precisionStopEnabled) {

            if (currentMovement.isComplete(pose)) {
                chassis.setDrivePower(0, 0, 0, dt);
                currentMovement = null;
            }
            else {
                currentMovement = currentMovement.maybeReplan(pose);
                driveToPose(currentMovement.getTarget(pose));
            }

            return;
        }

        Pose endPose = currentMovement.getEndPose();
        updateMode(endPose);

        if (mode == Mode.PRECISION) drivePrecisionMode(endPose);
        else {
            currentMovement = currentMovement.maybeReplan(pose);
            driveToPose(currentMovement.getTarget(pose));
        }
    }

    private void updateMode(Pose endPose) {

        final double positionDistance = Math.hypot(endPose.x - pose.x, endPose.y - pose.y);
        final double speed = Math.hypot(velocity.x, velocity.y);
        final double headingError = Math.abs(MathHelper.normalizeAngleRad(pose.heading - endPose.heading));
        final double angularSpeed = Math.abs(velocity.heading);

        //mode switching
        if (mode == Mode.TRANSIT) {

            boolean withinEntry = positionDistance < precisionModeThresholds.getEntryPositionDistance()
                    && speed < precisionModeThresholds.getEntryVelocity()
                    && headingError < precisionModeThresholds.getEntryHeadingError()
                    && angularSpeed < precisionModeThresholds.getEntryAngularVelocity();

            if (withinEntry) mode = Mode.PRECISION;
        }
        else {

            boolean pastExit = positionDistance > precisionModeThresholds.getExitPositionDistance()
                    || speed > precisionModeThresholds.getExitVelocity()
                    || headingError > precisionModeThresholds.getExitHeadingError()
                    || angularSpeed > precisionModeThresholds.getExitAngularVelocity();

            if (pastExit) mode = Mode.TRANSIT;
        }
    }

    private void driveToPose(Pose target) {

        double dx = target.x - pose.x;
        double dy = target.y - pose.y;
        double remainingDistance = Math.hypot(dx, dy);

        double fieldAngle = FastMath.atan2(dy, dx);
        double robotFrameAngle = MathHelper.normalizeAngleRad(fieldAngle - pose.heading);

        double closingVelocity = velocity.x * Math.cos(fieldAngle) + velocity.y * Math.sin(fieldAngle);

        double translationDecelMax = mecanumProfile.getMaxDeceleration(robotFrameAngle);
        double translationStoppingDistance = closingVelocity > 0
                ? (closingVelocity * closingVelocity) / (2d * translationDecelMax)
                : 0;

        boolean translationAccelerating = remainingDistance > translationStoppingDistance;
        double translationSign = translationAccelerating ? 1 : -1;

        double desiredForward = translationSign * Math.cos(robotFrameAngle);
        double desiredStrafe = -translationSign * Math.sin(robotFrameAngle);

        double headingError = MathHelper.normalizeAngleRad(target.heading - pose.heading);
        double remainingHeading = Math.abs(headingError);
        double headingDirection = Math.signum(headingError);

        double closingAngularVelocity = velocity.heading * headingDirection;
        double headingStoppingAngle = closingAngularVelocity > 0
                ? (closingAngularVelocity * closingAngularVelocity) / (2d * motionConstraints.getDmaxH())
                : 0;

        boolean headingAccelerating = remainingHeading > headingStoppingAngle;
        double desiredTurn = (headingAccelerating ? 1 : -1) * headingDirection;

        chassis.setDrivePower(desiredForward, desiredStrafe, desiredTurn, dt);
    }

    private void drivePrecisionMode(Pose target) {

        double fieldErrorX = pose.x - target.x;
        double fieldErrorY = pose.y - target.y;

        double forwardError = fieldErrorX * Math.cos(pose.heading) + fieldErrorY * Math.sin(pose.heading);
        double strafeError = fieldErrorX * Math.sin(pose.heading) - fieldErrorY * Math.cos(pose.heading);

        double forwardVelocity = velocity.x * Math.cos(pose.heading) + velocity.y * Math.sin(pose.heading);
        double strafeVelocity = velocity.x * Math.sin(pose.heading) - velocity.y * Math.cos(pose.heading);

        double headingError = MathHelper.normalizeAngleRad(pose.heading - target.heading);

        double forwardCorrection = poseLQR.correctForward(forwardError, forwardVelocity);
        double strafeCorrection = poseLQR.correctStrafe(strafeError, strafeVelocity);
        double headingCorrection = poseLQR.correctHeading(headingError, velocity.heading);

        double forwardPower = MathHelper.clamp(forwardCorrection / mecanumProfile.getMaxAcceleration(0), -1, 1);
        double strafePower = MathHelper.clamp(strafeCorrection / mecanumProfile.getMaxAcceleration(Math.PI / 2d), -1, 1);
        double turnPower = MathHelper.clamp(headingCorrection / motionConstraints.getAmaxH(), -1, 1);

        chassis.setDrivePower(forwardPower, strafePower, turnPower, dt);
    }

    public Chassis getChassis() {
        return chassis;
    }

    public FinalLocalizer getFinalLocalizer() {
        return localizer;
    }

    public MotionConstraints getMotionConstraints() {
        return motionConstraints;
    }

    public MecanumProfile getMecanumProfile() {
        return mecanumProfile;
    }

    public boolean isOnTransitMode() {
        return mode == Mode.TRANSIT;
    }

    public boolean isOnPrecisionMode() {
        return mode == Mode.PRECISION;
    }

    /// @return if the robot isn't following a path or if precision mode has
    /// taken over
    public boolean hasSettled() {
        return mode == Mode.PRECISION || currentMovement == null;
    }

    /// @return whether the robot is currently following a {@link Movement}
    public boolean isFollowing() {
        return currentMovement != null;
    }

}
