package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.util.KinematicState;
import org.firstinspires.ftc.teamcode.util.NoiseKalmanFilter;
import org.firstinspires.ftc.teamcode.util.Pose;

public class FinalLocalizer implements KinematicState {

    // heh heh heh heh, SIUUUUUUUUUUUUUUUUUUUUUU!!!

    private Pose pose, velocity, acceleration;
    private Pose prevVelocity, prevAcceleration;
    private double deltaTime;

    private final Localizer localizer;

    public final NoiseKalmanFilter velocityX, velocityY, velocityHeading;
    public final NoiseKalmanFilter accelerationX, accelerationY, accelerationHeading;

    public FinalLocalizer(Localizer localizer, boolean isBeingTuned) {

        pose = new Pose(0,0,0);
        velocity = new Pose(0,0,0);
        acceleration = new Pose(0,0,0);

        prevVelocity = new Pose(0,0,0);
        prevAcceleration = new Pose(0,0,0);

        deltaTime = 0;

        this.localizer = localizer;

        velocityX = new NoiseKalmanFilter(true);
        velocityY = new NoiseKalmanFilter(true);
        velocityHeading = new NoiseKalmanFilter(true);

        accelerationX = new NoiseKalmanFilter(true);
        accelerationY = new NoiseKalmanFilter(true);
        accelerationHeading = new NoiseKalmanFilter(true);

        areParametersSet = false;

        if (isBeingTuned) {

            velocityX.setTuning(true);
            velocityY.setTuning(true);
            velocityHeading.setTuning(true);

            accelerationX.setTuning(true);
            accelerationY.setTuning(true);
            accelerationHeading.setTuning(true);
        }
    }

    public FinalLocalizer(Localizer localizer) {
        this (localizer, false);
    }

    /// Params are provided as "new double[] {q, r, outlierThresholdMultiplier}"
    public void setNoiseFilterParameters(
            double[] velocityXParams,
            double[] velocityYParams,
            double[] velocityHeadingParams,
            double[] accelerationXParams,
            double[] accelerationYParams,
            double[] accelerationHeadingParams
    ) {

        setNKFParams(velocityX, velocityXParams);
        setNKFParams(velocityY, velocityYParams);
        setNKFParams(velocityHeading, velocityHeadingParams);

        setNKFParams(accelerationX, accelerationXParams);
        setNKFParams(accelerationY, accelerationYParams);
        setNKFParams(accelerationHeading, accelerationHeadingParams);

        areParametersSet = true;
    }

    private boolean areParametersSet;

    private void setNKFParams(NoiseKalmanFilter nkf, double[] params) {
        nkf.setParameters(params[0], params[1], params[2]);
    }

    public void setPose(Pose pose) {
        localizer.setPose(pose);
    }

    public void update() {

        if (!areParametersSet) throw new RuntimeException("Parameters weren't set!");

        localizer.update();

        deltaTime = localizer.getDeltaTime();

        //pose
        pose = localizer.getPose();

        //velocity
        Pose velocityRaw = localizer.getVelocity();
        velocityX.update(velocityRaw.x, deltaTime);
        velocityY.update(velocityRaw.y, deltaTime);
        velocityHeading.update(velocityRaw.heading, deltaTime);

        velocity = new Pose(velocityX.getOutput(), velocityY.getOutput(), velocityHeading.getOutput());

        //acceleration
        Pose accelerationRaw = velocity.minus(prevVelocity).divideBy(deltaTime);
        accelerationX.update(accelerationRaw.x, deltaTime);
        accelerationY.update(accelerationRaw.y, deltaTime);
        accelerationHeading.update(accelerationRaw.heading, deltaTime);

        acceleration = new Pose(accelerationX.getOutput(), accelerationY.getOutput(), accelerationHeading.getOutput());

        prevVelocity = velocity;
        prevAcceleration = acceleration;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public Pose getAcceleration() {
        return acceleration;
    }

    public Localizer getLocalizer() {
        return localizer;
    }

}
