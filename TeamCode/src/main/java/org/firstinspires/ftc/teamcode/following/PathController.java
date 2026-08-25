package org.firstinspires.ftc.teamcode.following;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PathController {

    private final Chassis chassis;

    private final FinalLocalizer localizer;
    private Pose pose, velocity, acceleration, jerk;

    private double dt;

    private final MotionConstraints motionConstraints;
    private final MecanumProfile mecanumProfile;

    private Movement currentMovement;

    public PathController(Chassis chassis, FinalLocalizer localizer, MotionConstraints motionConstraints) {

        this.chassis = chassis;

        this.localizer = localizer;

        this.motionConstraints = motionConstraints;
        mecanumProfile = this.motionConstraints.makeMecanumProfile();
    }

    /// Call once to start following a path.
    public void follow(Movement movement) {
        currentMovement = movement;
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

        if (currentMovement.isComplete(pose)) {
            chassis.setDrivePower(0, 0, 0, dt);
            currentMovement = null;
            return;
        }

        driveToPose(currentMovement.getTarget(pose));
    }

    public void driveToPose(Pose target) {

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
        double desiredStrafe = translationSign * Math.sin(robotFrameAngle);

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

    public Chassis getChassis() {
        return chassis;
    }

    public FinalLocalizer getFinalLocalizer() {
        return localizer;
    }
}