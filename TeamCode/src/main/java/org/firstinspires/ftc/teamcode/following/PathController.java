package org.firstinspires.ftc.teamcode.following;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.following.chassis.BrakingModel;
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

    /// Share of the wheels a rotation's position hold may take while the turn is braking.
    private static final double ROTATION_HOLD_BRAKE_AUTHORITY = 0.15;

    private static final int CURVE_LOOKAHEAD_STEPS = 6;

    private final Chassis chassis;
    private final MotionConstraints motionConstraints;
    private final MecanumProfile mecanumProfile;
    private final BrakingModel brakingModel;

    private final FinalLocalizer localizer;
    private Pose pose = new Pose(), velocity = new Pose(), acceleration = new Pose();

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

    public PathController(Chassis chassis, FinalLocalizer localizer, MotionConstraints motionConstraints, BrakingModel brakingModel, PoseLQRController poseLQR, PrecisionModeThresholds precisionModeThresholds) {

        this.chassis = chassis;

        this.localizer = localizer;

        this.motionConstraints = motionConstraints;
        mecanumProfile = this.motionConstraints.makeMecanumProfile();

        this.brakingModel = brakingModel;
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

        if (movement != drivenMovement) {

            drivenMovement = movement;
            hasPreviousTargetHeading = false;
        }

        //a rotation only holds its spot through the LQR, precisionStop won't disable it
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

            final double normalX = -tangent.y;
            final double normalY = tangent.x;

            double alongCommand = 0;

            if (tangent.x != 0 || tangent.y != 0) {

                double alongVelocity = velocity.x * tangent.x + velocity.y * tangent.y;
                double alongAngle = robotFrameAngle(tangent.x, tangent.y);

                //whichever of the two wants less throttle wins, so a bend can only ever slow it down
                alongCommand = Math.min(
                        axisCommand(movement.getRemainingDistance(pose), alongVelocity, alongAngle),
                        curveCommand(movement, alongVelocity, alongAngle, robotFrameAngle(normalX, normalY))
                );
            }

            double crossCommand = axisCommand(
                    movement.getSignedCrossTrack(pose),
                    velocity.x * normalX + velocity.y * normalY,
                    robotFrameAngle(normalX, normalY)
            );

            double driveX = alongCommand * tangent.x + crossCommand * normalX;
            double driveY = alongCommand * tangent.y + crossCommand * normalY;

            desiredForward = 0;
            desiredStrafe = 0;

            if (driveX != 0 || driveY != 0) {

                double driveAngle = robotFrameAngle(driveX, driveY);

                desiredForward = Math.cos(driveAngle);
                desiredStrafe = -Math.sin(driveAngle);
            }
        }

        double desiredTurn;

        if (headingMode == Mode.PRECISION) desiredTurn = lqrTurn;
        else desiredTurn = headingCommand(MathHelper.normalizeAngleRad(target.heading - pose.heading), velocity.heading);

        //the turn's braking distance assumes it owns the wheels, so holding yields while it brakes
        if (movement instanceof Rotation && desiredTurn * velocity.heading < 0) {

            double magnitude = Math.hypot(desiredForward, desiredStrafe);

            if (magnitude > ROTATION_HOLD_BRAKE_AUTHORITY) {
                desiredForward *= ROTATION_HOLD_BRAKE_AUTHORITY / magnitude;
                desiredStrafe *= ROTATION_HOLD_BRAKE_AUTHORITY / magnitude;
            }
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

    private double axisCommand(double error, double closingVelocity, double axisAngle) {

        final double margin = brakingModel.getMargin();

        double stoppingDistance = brakingModel.getStoppingDistance(axisAngle, Math.abs(closingVelocity));

        //inside the margin and able to stop inside it, so there is nothing worth commanding
        if (Math.abs(error) < margin && stoppingDistance < margin) return 0;

        boolean movingTowardTarget = error * closingVelocity > 0;

        if (movingTowardTarget && Math.abs(error) <= stoppingDistance) return -Math.signum(closingVelocity);

        return Math.signum(error);
    }

    private double curveCommand(Movement movement, double alongVelocity, double alongAngle, double normalAngle) {

        //a bend can only be held at v = sqrt(sideways grip / curvature), so the path ahead is
        //checked for bends the robot couldn't still slow down for in time

        if (alongVelocity <= 0) return 1;

        final double grip = mecanumProfile.getMaxAcceleration(normalAngle);
        final double cut = brakingModel.getMargin();
        final double reach = brakingModel.getStoppingDistance(alongAngle, alongVelocity);
        final double step = reach / CURVE_LOOKAHEAD_STEPS;

        double command = 1;
        double curvature = movement.getCurvature(pose, 0, cut);

        for (int i = 0; i <= CURVE_LOOKAHEAD_STEPS; i++) {

            double ahead = step * i;
            double next = movement.getCurvature(pose, ahead + step, cut);

            double allowed = cornerSpeed(curvature, step > 0 ? Math.abs(next - curvature) / step : 0, grip);

            curvature = next;

            if (alongVelocity <= allowed) continue;

            //shedding speed costs the difference between the two stopping distances
            if (reach - brakingModel.getStoppingDistance(alongAngle, allowed) >= ahead) return -1;

            command = 0;
        }

        return command;
    }

    /*
     * Twinkle twinkle little star,
     * How I wonder what you are!
     * Up-in-pathing-algorithm-land-I'm-feeling-very-happy-because-this-stuff-is-actually-so-revolutionary-and-I'm-so-excited!
     */

    private double cornerSpeed(double curvature, double curvatureRate, double grip) {

        //uses grip, turning, and acceleration to keep the robot on track

        if (curvature <= 0) return Double.MAX_VALUE;

        double speed = Math.min(Math.sqrt(grip / curvature), motionConstraints.getVmaxH() / curvature);

        if (curvatureRate <= 0) return speed;

        return Math.min(speed, Math.sqrt(motionConstraints.getAmaxH() / curvatureRate));
    }

    private double headingCommand(double error, double angularVelocity) {

        final double margin = brakingModel.getAngularMargin();

        double stoppingAngle = brakingModel.getAngularStoppingDistance(Math.abs(angularVelocity));

        if (Math.abs(error) < margin && stoppingAngle < margin) return 0;

        boolean turningTowardTarget = error * angularVelocity > 0;

        if (turningTowardTarget && Math.abs(error) <= stoppingAngle) return -Math.signum(angularVelocity);

        return Math.signum(error);
    }

    private double robotFrameAngle(double axisX, double axisY) {
        return MathHelper.normalizeAngleRad(FastMath.atan2(axisY, axisX) - pose.heading);
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

    public BrakingModel getBrakingModel() {
        return brakingModel;
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

    /// @return if the maneuver isn't running or if precision mode has taken over
    public boolean hasSettled() {
        return !isFollowing() || isOnPrecisionMode();
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
