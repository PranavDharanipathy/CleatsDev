package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class Movement {

    public abstract Pose getTarget(Pose currentPose);

    public abstract boolean isComplete(Pose currentPose);

    public abstract Pose getEndPose();

    private Replanner replanner;
    private double replanOffShootDistance;

    /// Adds an optional replanning algorithm. When the robot's path error
    /// determined by {@link #getPathError} exceeds offShootDistance, the
    /// movement is replanned.
    public Movement setReplanner(Replanner replanner, double offShootDistance) {
        
        this.replanner = replanner;
        this.replanOffShootDistance = offShootDistance;

        return this;
    }

    protected double getReplanOffShootDistance() {
        return replanOffShootDistance;
    }

    public boolean isReplanEnabled() {
        return replanner != null;
    }

    /// Field frame unit vector the robot should translate along, with the heading
    /// parameter ignored. Defaults to pointing straight at the target.
    public Pose getDriveDirection(Pose currentPose) {

        Pose target = getTarget(currentPose);
        return toUnitVector(target.x - currentPose.x, target.y - currentPose.y);
    }

    /// Distance left to travel, used to decide when to start braking.
    public double getRemainingDistance(Pose currentPose) {

        Pose end = getEndPose();
        return Math.hypot(end.x - currentPose.x, end.y - currentPose.y);
    }

    protected static Pose toUnitVector(double x, double y) {

        double magnitude = Math.hypot(x, y);
        return magnitude > 0 ? new Pose(x / magnitude, y / magnitude, 0) : new Pose(0, 0, 0);
    }

    /// How far the robot currently is from the ideal path.
    /// <p>
    /// Note that any {@link Movement} that supports replanning overrides this method.
    /// <p>
    /// Default is 0 meaning it's "never off path".
    public double getPathError(Pose currentPose) {
        return 0;
    }

    public Movement maybeReplan(Pose currentPose) { //only matters if replanning is enabled.

        if (replanner == null) return this;
        if (getPathError(currentPose) <= replanOffShootDistance) return this;

        Movement replanned = replanner.replan(currentPose);
        return replanned != null ? replanned : this;
    }
}
