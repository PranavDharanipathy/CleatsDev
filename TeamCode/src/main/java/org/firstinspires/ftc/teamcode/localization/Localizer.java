package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class Localizer {

    protected Pose pose, velocity;

    protected double deltaTime;

    private boolean firstUpdate = true;
    private double previousTime, currentTime;

    public Localizer() {

        pose = new Pose();
        velocity = new Pose();

        deltaTime = 0;
    }

    public abstract void setPose(Pose pose);

    public abstract void update();

    /// The first call reports 0 so the gap since construction isn't read as a loop.
    protected void updateDeltaTime() {

        previousTime = currentTime;
        currentTime = System.nanoTime() * 1e-9;

        if (firstUpdate) {

            firstUpdate = false;
            deltaTime = 0;

            return;
        }

        deltaTime = currentTime - previousTime;
    }

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
