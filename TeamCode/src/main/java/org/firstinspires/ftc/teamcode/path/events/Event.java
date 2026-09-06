package org.firstinspires.ftc.teamcode.path.events;

public abstract class Event {

    private final Runnable task;
    private boolean hasRun;

    public Event(Runnable task) {
        this.task = task;
    }

    /// @param progress 0 to 1 along the movement this event belongs to
    /// @param travelledDistance inches covered along that movement
    /// @param remainingDistance inches left along that movement
    /// @param movementComplete whether the movement has just finished
    public abstract boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete);

    public void poll(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {

        if (hasRun || task == null) return;
        if (!executionCondition(progress, travelledDistance, remainingDistance, movementComplete)) return;

        hasRun = true;
        task.run();
    }

    public void reset() {
        hasRun = false;
    }

    public boolean hasRun() {
        return hasRun;
    }
}
