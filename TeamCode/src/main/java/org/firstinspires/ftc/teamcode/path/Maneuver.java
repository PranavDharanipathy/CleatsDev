package org.firstinspires.ftc.teamcode.path;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.path.events.Event;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

/// A chain of movements followed as one continuous motion. Precision only engages
/// at the end of a movement that is waited on, followed by a rotation, last, or
/// otherwise specified.
public class Maneuver {

    private static class Step {

        Movement movement, originalMovement;
        Event event;
        Boolean precisionStop;
        double waitSeconds;

        double length;
        double storedDistance;
        boolean waiting;
        double waitElapsed;

        double total() {
            return storedDistance + length;
        }

        double travelled(Pose pose) {
            return Math.max(0, total() - movement.getRemainingDistance(pose));
        }

        double progress(Pose pose) {
            return total() > 0 ? MathHelper.clamp(travelled(pose) / total(), 0, 1) : 0;
        }
    }

    private final List<Step> steps = new ArrayList<>();

    private int index;
    private boolean started, finished, cancelled;

    private double completedLength;
    private double travelledAtStop;

    private Pose lastPose;

    public Maneuver addMovement(Movement movement) {
        return add(movement, null, null);
    }

    public Maneuver addMovement(Movement movement, Event event) {
        return add(movement, event, null);
    }

    /// @param precisionStop overrides the LQR handoff for this movement only, ignored by rotations
    public Maneuver addMovement(Movement movement, Event event, boolean precisionStop) {
        return add(movement, event, precisionStop);
    }

    private Maneuver add(Movement movement, Event event, Boolean precisionStop) {

        Step step = new Step();

        step.movement = movement;
        step.originalMovement = movement;
        step.event = event;
        step.precisionStop = precisionStop;

        steps.add(step);

        return this;
    }

    /// Holds the end pose for this long once the movement before it finishes.
    public Maneuver waitSeconds(double seconds) {

        if (!steps.isEmpty()) steps.get(steps.size() - 1).waitSeconds = seconds;

        return this;
    }

    private boolean usesPrecision(int i) {

        Step step = steps.get(i);

        //a rotation covers no ground and is held in place by the LQR alone, so this is not overridable
        if (step.movement instanceof Rotation) return true;

        if (step.precisionStop != null) return step.precisionStop;
        if (step.waitSeconds > 0) return true;
        if (i == steps.size() - 1) return true;

        return steps.get(i + 1).movement instanceof Rotation;
    }

    //walks the chain so unreached steps still count toward the total
    private void measure(Pose currentPose) {

        Pose from = currentPose;

        for (Step step : steps) {

            if (step.movement instanceof Rotation) {
                step.length = 0;
                continue;
            }

            step.length = step.movement.getRemainingDistance(from);
            from = step.movement.getEndPose();
        }
    }

    /// Advances the chain and returns the movement to drive, or null once nothing is left.
    public Movement update(Pose currentPose, double handoffDistance, double dt) {

        lastPose = currentPose;

        if (finished || cancelled || steps.isEmpty()) return null;

        if (!started) {

            measure(currentPose);
            started = true;
        }

        Step step = steps.get(index);

        Movement replanned = step.movement.maybeReplan(currentPose);

        if (replanned != step.movement) {

            step.storedDistance += Math.max(0, step.length - step.movement.getRemainingDistance(currentPose));
            step.movement = replanned;
            step.length = replanned.getRemainingDistance(currentPose);
        }

        boolean settles = usesPrecision(index) || handoffDistance <= 0;

        double remaining = step.movement.getRemainingDistance(currentPose);

        boolean movementComplete = settles
                ? step.movement.isComplete(currentPose)
                : remaining <= handoffDistance;

        if (step.event != null) step.event.poll(step.progress(currentPose), step.travelled(currentPose), remaining, movementComplete);

        //once the wait starts it keeps running, even if the robot drifts back out of the window
        if (!step.waiting && !movementComplete) return step.movement;

        if (step.waitSeconds > 0) {

            step.waiting = true;
            step.waitElapsed += Math.max(0, dt);

            if (step.waitElapsed < step.waitSeconds) return step.movement;
        }

        completedLength += step.total();
        index++;

        if (index >= steps.size()) {

            finished = true;
            travelledAtStop = getLength();

            return null;
        }

        return steps.get(index).movement;
    }

    /// Rewinds so the same object can be followed again.
    public void reset() {

        index = 0;
        started = false;
        finished = false;
        cancelled = false;
        completedLength = 0;
        travelledAtStop = 0;
        lastPose = null;

        for (Step step : steps) {

            step.storedDistance = 0;
            step.waiting = false;
            step.waitElapsed = 0;

            step.movement = step.originalMovement;
            step.movement.reset();

            if (step.event != null) step.event.reset();
        }
    }

    /// Stops the chain where it is. Distance and percent hold rather than reading as finished.
    public void cancel() {

        if (!isFollowing()) return;

        travelledAtStop = getTravelledDistance();
        cancelled = true;
    }

    public boolean isFollowing() {
        return !finished && !cancelled && !steps.isEmpty();
    }

    public boolean isCancelled() {
        return cancelled;
    }

    /// Whether the movement being driven right now is allowed to hand off to the LQR.
    public boolean isPrecisionAllowed() {
        return isFollowing() && usesPrecision(index);
    }

    /// Whether the chain is holding an end pose while a waitSeconds runs down.
    public boolean isWaiting() {
        return isFollowing() && steps.get(index).waiting;
    }

    public boolean isOnLastMovement() {
        return isFollowing() && index == steps.size() - 1;
    }

    public Movement getCurrentMovement() {
        return isFollowing() ? steps.get(index).movement : null;
    }

    public int getCurrentIndex() {
        return index;
    }

    public int getMovementCount() {
        return steps.size();
    }

    public double getLength() {

        if (!started) return 0;

        double total = completedLength;

        for (int i = index; i < steps.size(); i++) total += steps.get(i).total();

        return total;
    }

    public double getTravelledDistance() {

        if (!isFollowing() || lastPose == null) return travelledAtStop;

        return completedLength + steps.get(index).travelled(lastPose);
    }

    public double getRemainingDistance() {
        return Math.max(0, getLength() - getTravelledDistance());
    }

    public double getPercent() {

        double length = getLength();

        if (length <= 0) return finished ? 1 : 0;

        return MathHelper.clamp(getTravelledDistance() / length, 0, 1);
    }
}
