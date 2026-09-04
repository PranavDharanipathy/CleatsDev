package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.Pose;

/// Replans {@link Movement} when robot ends up off path due to collisions or other factors.
@FunctionalInterface
public interface Replanner {

    /// @param currentPose the robot's current pose
    /// @return a new Movement to follow or to keep following the current one
    Movement replan(Pose currentPose);
}
