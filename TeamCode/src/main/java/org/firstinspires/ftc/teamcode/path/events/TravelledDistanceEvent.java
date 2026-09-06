package org.firstinspires.ftc.teamcode.path.events;

public class TravelledDistanceEvent extends Event {

    private final double distanceThreshold;

    /// @param distanceThreshold inches along the movement it is attached to
    public TravelledDistanceEvent(double distanceThreshold, Runnable task) {

        super(task);
        this.distanceThreshold = distanceThreshold;
    }

    @Override
    public boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {
        return travelledDistance >= distanceThreshold;
    }
}
