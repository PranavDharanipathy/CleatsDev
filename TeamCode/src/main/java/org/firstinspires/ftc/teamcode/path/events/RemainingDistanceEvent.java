package org.firstinspires.ftc.teamcode.path.events;

public class RemainingDistanceEvent extends Event {

    private final double distanceThreshold;

    /// @param distanceThreshold inches left along the movement it is attached to
    public RemainingDistanceEvent(double distanceThreshold, Runnable task) {

        super(task);
        this.distanceThreshold = distanceThreshold;
    }

    @Override
    public boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {
        return remainingDistance <= distanceThreshold;
    }
}
