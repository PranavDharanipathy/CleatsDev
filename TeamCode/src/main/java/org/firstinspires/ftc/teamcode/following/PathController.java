package org.firstinspires.ftc.teamcode.following;

import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.path.Movement;

public class PathController {

    // calma calma...

    private final Chassis chassis;

    private final FinalLocalizer localizer;

    private final MotionConstraints motionConstraints;
    private final MecanumProfile mecanumProfile;

    private Pose pose, velocity, acceleration, jerk;

    private double dt;

    public PathController(Chassis chassis, FinalLocalizer localizer, MotionConstraints motionConstraints) {

        this.chassis = chassis;

        this.localizer = localizer;

        this.motionConstraints = motionConstraints;
        mecanumProfile = this.motionConstraints.makeMecanumProfile();
    }

    public void follow(Movement movement) {

    }

    public void update() {

        localizer.update();

        dt = localizer.getDeltaTime();

        pose = localizer.getPose();
        velocity = localizer.getVelocity();
        acceleration = localizer.getAcceleration();
        jerk = localizer.getJerk();
    }

    public Chassis getChassis() {
        return chassis;
    }

    public FinalLocalizer getFinalLocalizer() {
        return localizer;
    }
}
