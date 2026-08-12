package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class Localizer {

    protected Pose pose, velocity;

    protected double deltaTime;

    public Localizer() {

        pose = new Pose(0,0,0);
        velocity = new Pose(0,0,0);

        deltaTime = 0;
    }

    public abstract void setPose(Pose pose);

    public abstract void update();

    public Pose getPose() {
        return pose;
    }

    public Pose getVelocity() {
        return velocity;
    }

    /// @return time between localizer updates in seconds.
    public double getDeltaTime() {
        return deltaTime;
    }

}