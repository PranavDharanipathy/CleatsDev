package org.firstinspires.ftc.teamcode.dribble;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

/// Hooks a command onto a condition.
/// <p>
/// Starts as soon as it's made and the scheduler polls it each loop.
public class Trigger {

    private final BooleanSupplier condition;
    private final ArrayList<Watch> watches = new ArrayList<>();

    private boolean pastState;

    public Trigger(BooleanSupplier condition) {

        this.condition = condition;
        this.pastState = condition.getAsBoolean();

        CommandScheduler.register(this);
    }

    /// Schedules once each time the condition goes from false to true
    public Trigger whenTrue(Command command) {
        return on(true, false, command, null);
    }

    /// Schedules once each time the condition goes from true to false
    public Trigger whenFalse(Command command) {
        return on(false, false, command, null);
    }

    /// Schedules on the way up and cancels on the way down
    public Trigger whileTrue(Command command) {
        return on(true, false, command, command);
    }

    /// Schedules on the way up, and the next way up cancels it instead
    public Trigger toggleWhenTrue(Command command) {
        return on(true, true, command, null);
    }

    public Trigger and(BooleanSupplier other) {
        return new Trigger(() -> condition.getAsBoolean() && other.getAsBoolean());
    }

    public Trigger or(BooleanSupplier other) {
        return new Trigger(() -> condition.getAsBoolean() || other.getAsBoolean());
    }

    public Trigger negate() {
        return new Trigger(() -> !condition.getAsBoolean());
    }

    void poll() {

        boolean currentState = condition.getAsBoolean();

        if (currentState == pastState) return;

        for (Watch watch : watches) {

            if (currentState == watch.edge) {

                if (watch.toggle && watch.schedule.isScheduled()) watch.schedule.cancel();
                else CommandScheduler.schedule(watch.schedule);
            }
            else if (watch.cancel != null) watch.cancel.cancel();
        }

        pastState = currentState;
    }

    private Trigger on(boolean edge, boolean toggle, Command schedule, Command cancel) {

        watches.add(new Watch(edge, toggle, schedule, cancel));
        return this;
    }

    private static final class Watch {

        final boolean edge, toggle;
        final Command schedule, cancel;

        Watch(boolean edge, boolean toggle, Command schedule, Command cancel) {

            this.edge = edge;
            this.toggle = toggle;
            this.schedule = schedule;
            this.cancel = cancel;
        }
    }
}
