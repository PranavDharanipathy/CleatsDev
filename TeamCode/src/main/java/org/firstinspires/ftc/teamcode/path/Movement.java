package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class Movement {

    public abstract Pose getTarget(Pose currentPose);

    public abstract boolean isComplete(Pose currentPose);

    public abstract Pose getEndPose();
}