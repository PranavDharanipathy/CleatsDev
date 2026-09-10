package org.firstinspires.ftc.teamcode.following;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.path.Maneuver;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.path.Rotation;

public class PathController {

    // Wait,
    // Run that back,

    // Nothing ever goes wrong in Cleats.

    public enum Mode {
        TRANSIT, PRECISION
    }

    private final Chassis chassis;
    private final MotionConstraints motionConstraints;
    private final MecanumProfile mecanumProfile;

    private final FinalLocalizer localizer;
    private Pose pose, velocity, acceleration;

    private double dt;

    private final PoseLQRController poseLQR;
    private final PrecisionModeThresholds precisionModeThresholds;

    private Maneuver currentManeuver;
    private boolean precisionStopEnabled;
    private Movement drivenMovement;

    //translation and heading have independent LQR management
    private Mode translationMode, headingMode;

    private double previousTargetHeading;
    private boolean hasPreviousTargetHeading;

    public PathController(Chassis chassis, FinalLocalizer localizer, MotionConstraints motionConstraints, PoseLQRController poseLQR, PrecisionModeThresholds precisionModeThresholds) {

        this.chassis = chassis;

        this.localizer = localizer;

        this.motionConstraints = motionConstraints;
        mecanumProfile = this.motionConstraints.makeMecanumProfile();

        this.poseLQR = poseLQR;
        this.precisionModeThresholds = precisionModeThresholds;

        translationMode = Mode.TRANSIT;
        headingMode = Mode.TRANSIT;
    }

    /// Call once to start following a path.
    public void follow(Movement movement) {
        follow(movement, true);
    }

    /// Call once to start following a path.
    /// @param precisionStop whether to use precision mode for end pose correction or not, ignored by rotations
    public void follow(Movement movement, boolean precisionStop) {
        follow(new Maneuver().addMovement(movement, null, precisionStop), precisionStop);
    }

    /// Call once to start following a series of movements.
    public void follow(Maneuver maneuver) {
        follow(maneuver, true);
    }

    /// Call once to start following a series of movements.
    /// @param precisionStop turns precision mode off for the whole maneuver, ignored by rotations
    public void follow(Maneuver maneuver, boolean precisionStop) {

        currentManeuver = maneuver;
        currentManeuver.reset();

        precisionStopEnabled = precisionStop;

        translationMode = Mode.TRANSIT;
        headingMode = Mode.TRANSIT;

        drivenMovement = null;
        hasPreviousTargetHeading = false;
    }

    /// Cancels the whole maneuver and stops the drivetrain. Statistics such
    /// as traveled distance, remaining distance, and maneuver percent stay
    /// what they were until a new {@link Maneuver} is followed.
    public void cancel() {

        if (currentManeuver == null) return;

        currentManeuver.cancel();

        translationMode = Mode.TRANSIT;
        headingMode = Mode.TRANSIT;

        chassis.setDrivePowerBypassRamp(0, 0, 0);
    }

    /// Must be called every loop.
    public void update() {

        localizer.update();

        dt = localizer.getDeltaTime();

        pose = localizer.getPose();
        velocity = localizer.getVelocity();
        acceleration = localizer.getAcceleration();

        if (currentManeuver == null || !currentManeuver.isFollowing()) return;

        Movement movement = currentManeuver.update(pose, precisionModeThresholds.getEntryPositionDistance(), dt);

        if (movement == null) {

            chassis.setDrivePowerBypassRamp(0, 0, 0);
            return;
        }

        //a new movement means a new heading target, so the feedforward can't carry over
        if (movement != drivenMovement) {

            drivenMovement = movement;
            hasPreviousTargetHeading = false;
        }

        //a rotation only holds its spot through the LQR, so precisionStop can't switch it off
        boolean precisionStop = currentManeuver.isPrecisionAllowed()
                && (precisionStopEnabled || movement instanceof Rotation);

        drive(movement, movement.getTarget(pose), movement.getEndPose(), precisionStop);
    }

    private void drive(Movement movement, Pose target, Pose endPose, boolean precisionStop) {

        double lqrForward = 0, lqrStrafe = 0, lqrTurn = 0;

        if (precisionStop) {

            //target moves along the path, so LQR uses the rate of error instead of plain angular velocity
            double targetHeadingRate = 0;

            if (hasPreviousTargetHeading && dt > 0) targetHeadingRate = MathHelper.normalizeAngleRad(target.heading - previousTargetHeading) / dt;

            previousTargetHeading = target.heading;
            hasPreviousTargetHeading = true;

            double fieldErrorX = pose.x - endPose.x;
            double fieldErrorY = pose.y - endPose.y;

            double forwardError = fieldErrorX * Math.cos(pose.heading) + fieldErrorY * Math.sin(pose.heading);
            double strafeError = fieldErrorX * Math.sin(pose.heading) - fieldErrorY * Math.cos(pose.heading);

            double forwardVelocity = velocity.x * Math.cos(pose.heading) + velocity.y * Math.sin(pose.heading);
            double strafeVelocity = velocity.x * Math.sin(pose.heading) - velocity.y * Math.cos(pose.heading);

            double headingError = MathHelper.normalizeAngleRad(pose.heading - target.heading);
            double relativeAngularVelocity = velocity.heading - targetHeadingRate;

            lqrForward = poseLQR.correctForward(forwardError, forwardVelocity) / mecanumProfile.getMaxAcceleration(0);
            lqrStrafe = poseLQR.correctStrafe(strafeError, strafeVelocity) / mecanumProfile.getMaxAcceleration(Math.PI / 2d);
            lqrTurn = poseLQR.correctHeading(headingError, relativeAngularVelocity) / motionConstraints.getAmaxH();

            //determining whether LQR or bang-bang should be used
            boolean translationAuthority = Math.abs(lqrForward) < 1 && Math.abs(lqrStrafe) < 1;
            boolean headingAuthority = Math.abs(lqrTurn) < 1;

            updateModes(target, endPose, translationAuthority, headingAuthority, relativeAngularVelocity);
        }
        else {

            hasPreviousTargetHeading = false;

            translationMode = Mode.TRANSIT;
            headingMode = Mode.TRANSIT;
        }

        double desiredForward, desiredStrafe;

        if (translationMode == Mode.PRECISION) {
            desiredForward = lqrForward;
            desiredStrafe = lqrStrafe;
        }
        else {

            Pose tangent = movement.getTangentDirection(pose);

            double alongRemaining = movement.getRemainingDistance(pose);
            double crossTrack = movement.getSignedCrossTrack(pose);

            double normalX = -tangent.y;
            double normalY = tangent.x;

            double alongCommand = 0;

            if (tangent.x != 0 || tangent.y != 0) {

                double closingVelocity = velocity.x * tangent.x + velocity.y * tangent.y;
                alongCommand = shouldAccelerateOrBrake(alongRemaining, closingVelocity, tangent.x, tangent.y);
            }

            double crossCommand = 0;
            double crossDirection = Math.signum(crossTrack);

            if (crossDirection != 0) {

                double closingVelocity = (velocity.x * normalX + velocity.y * normalY) * crossDirection;

                crossCommand = crossDirection * shouldAccelerateOrBrake(Math.abs(crossTrack), closingVelocity, normalX * crossDirection, normalY * crossDirection);
            }

            double driveX = alongCommand * tangent.x + crossCommand * normalX;
            double driveY = alongCommand * tangent.y + crossCommand * normalY;

            desiredForward = 0;
            desiredStrafe = 0;

            if (driveX != 0 || driveY != 0) {

                double robotFrameAngle = MathHelper.normalizeAngleRad(FastMath.atan2(driveY, driveX) - pose.heading);

                desiredForward = Math.cos(robotFrameAngle);
                desiredStrafe = -Math.sin(robotFrameAngle);
            }
        }

        double desiredTurn;

        if (headingMode == Mode.PRECISION) desiredTurn = lqrTurn;
        else {

            double headingError = MathHelper.normalizeAngleRad(target.heading - pose.heading);
            double remainingHeading = Math.abs(headingError);
            double headingDirection = Math.signum(headingError);

            double closingAngularVelocity = velocity.heading * headingDirection;
            double headingStoppingAngle = closingAngularVelocity > 0
                    ? (closingAngularVelocity * closingAngularVelocity) / (2d * motionConstraints.getDmaxH())
                    : 0;

            boolean headingAccelerating = remainingHeading > headingStoppingAngle;
            desiredTurn = (headingAccelerating ? 1 : -1) * headingDirection;
        }

        chassis.setDrivePower(desiredForward, desiredStrafe, desiredTurn, dt);
    }

    private void updateModes(Pose target, Pose endPose, boolean translationAuthority, boolean headingAuthority, double relativeAngularVelocity) {

        final double positionDistance = Math.hypot(endPose.x - pose.x, endPose.y - pose.y);
        final double speed = Math.hypot(velocity.x, velocity.y);

        if (translationMode == Mode.TRANSIT) {

            boolean withinEntry = positionDistance < precisionModeThresholds.getEntryPositionDistance()
                    && speed < precisionModeThresholds.getEntryVelocity()
                    && translationAuthority;

            if (withinEntry) translationMode = Mode.PRECISION;
        }
        else {

            boolean pastExit = positionDistance > precisionModeThresholds.getExitPositionDistance()
                    || speed > precisionModeThresholds.getExitVelocity()
                    || !translationAuthority;

            if (pastExit) translationMode = Mode.TRANSIT;
        }

        final double headingError = Math.abs(MathHelper.normalizeAngleRad(pose.heading - target.heading));
        final double angularSpeed = Math.abs(relativeAngularVelocity);

        if (headingMode == Mode.TRANSIT) {

            boolean withinEntry = headingError < precisionModeThresholds.getEntryHeadingError()
                    && angularSpeed < precisionModeThresholds.getEntryAngularVelocity()
                    && headingAuthority;

            if (withinEntry) headingMode = Mode.PRECISION;
        }
        else {

            boolean pastExit = headingError > precisionModeThresholds.getExitHeadingError()
                    || angularSpeed > precisionModeThresholds.getExitAngularVelocity()
                    || !headingAuthority;

            if (pastExit) headingMode = Mode.TRANSIT;
        }
    }

    //1 when there's room to keep speeding up, -1 once there's enough room left to stop
    private double shouldAccelerateOrBrake(double remaining, double closingVelocity, double axisX, double axisY) {

        double robotFrameAngle = MathHelper.normalizeAngleRad(FastMath.atan2(axisY, axisX) - pose.heading);

        double stoppingDistance = closingVelocity > 0
                ? (closingVelocity * closingVelocity) / (2d * mecanumProfile.getMaxDeceleration(robotFrameAngle))
                : 0;

        return remaining > stoppingDistance ? 1 : -1;
    }

    /// @return inches left in the whole maneuver, along the paths themselves
    public double getRemainingDistance() {
        return currentManeuver == null ? 0 : currentManeuver.getRemainingDistance();
    }

    /// @return inches covered across the whole maneuver, carried over through replans
    public double getTravelledDistance() {
        return currentManeuver == null ? 0 : currentManeuver.getTravelledDistance();
    }

    /// @return the whole maneuver's arc length, only growing if a replan routes further
    public double getPathLength() {
        return currentManeuver == null ? 0 : currentManeuver.getLength();
    }

    /// @return how much of the maneuver is done, 0 to 1
    public double getPathPercent() {
        return currentManeuver == null ? 0 : currentManeuver.getPercent();
    }

    public Maneuver getCurrentManeuver() {
        return currentManeuver;
    }

    public Movement getCurrentMovement() {
        return currentManeuver == null ? null : currentManeuver.getCurrentMovement();
    }

    public boolean isWaiting() {
        return currentManeuver != null && currentManeuver.isWaiting();
    }

    public boolean isOnLastMovement() {
        return currentManeuver != null && currentManeuver.isOnLastMovement();
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

    public boolean isTranslationOnPrecisionMode() {
        return translationMode == Mode.PRECISION;
    }

    public boolean isHeadingOnPrecisionMode() {
        return headingMode == Mode.PRECISION;
    }

    /// @return whether either axis is still running bang-bang
    public boolean isOnTransitMode() {
        return translationMode == Mode.TRANSIT || headingMode == Mode.TRANSIT;
    }

    /// @return whether both axes have handed off to the LQR
    public boolean isOnPrecisionMode() {
        return !isOnTransitMode();
    }

    public Mode getTranslationMode() {
        return translationMode;
    }

    public Mode getHeadingMode() {
        return headingMode;
    }

    public boolean hasSettled() {
        return !isFollowing() || (isOnLastMovement() && isOnPrecisionMode());
    }

    /// @return whether a maneuver is currently being followed
    public boolean isFollowing() {
        return currentManeuver != null && currentManeuver.isFollowing();
    }

    public double getX() {
        return pose.x;
    }

    public double getY() {
        return pose.y;
    }

    public double getHeading() {
        return pose.heading;
    }

    public Pose getPose() {
        return pose;
    }

    public Pose getVelocity() {
        return velocity;
    }

    public Pose getAcceleration() {
        return acceleration;
    }

}
