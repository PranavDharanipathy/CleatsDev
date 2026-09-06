package org.firstinspires.ftc.teamcode.path.events;

public class ProgressEvent extends Event {

    private final double progressThreshold;

    /// @param progressThreshold 0 to 1 along the movement it is attached to
    public ProgressEvent(double progressThreshold, Runnable task) {

        super(task);
        this.progressThreshold = progressThreshold;
    }

    @Override
    public boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {
        return progress >= progressThreshold;
    }
}
