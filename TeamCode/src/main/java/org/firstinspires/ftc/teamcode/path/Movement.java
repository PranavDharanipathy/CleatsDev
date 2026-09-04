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
