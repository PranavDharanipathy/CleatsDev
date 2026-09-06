package org.firstinspires.ftc.teamcode.path.events;

public class EndEvent extends Event {

    public EndEvent(Runnable task) {
        super(task);
    }

    @Override
    public boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {
        return movementComplete;
    }
}
