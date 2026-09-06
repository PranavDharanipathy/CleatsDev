package org.firstinspires.ftc.teamcode.path.events;

public class StartEvent extends Event {

    public StartEvent(Runnable task) {
        super(task);
    }

    @Override
    public boolean executionCondition(double progress, double travelledDistance, double remainingDistance, boolean movementComplete) {
        return true;
    }
}
