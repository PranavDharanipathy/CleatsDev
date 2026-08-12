package org.firstinspires.ftc.teamcode.following;

import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.path.Movement;

public class PathController {

    // calma calma...

    private final Chassis chassis;

    private final FinalLocalizer localizer;
    private Pose pose, velocity, acceleration, jerk;

    private double dt;

    public PathController(Chassis chassis, FinalLocalizer localizer) {

        this.chassis = chassis;

        this.localizer = localizer;
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
