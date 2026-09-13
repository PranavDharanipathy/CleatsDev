package org.firstinspires.ftc.teamcode.dribble;

import java.util.ArrayList;
import java.util.LinkedHashMap;

public final class CommandScheduler {

    private static final ArrayList<Command> running = new ArrayList<>();
    private static final ArrayList<Command> pending = new ArrayList<>();
    private static final ArrayList<Command> cancelling = new ArrayList<>();

    private static final LinkedHashMap<Subsystem, Command> subsystems = new LinkedHashMap<>();
    private static final ArrayList<Trigger> triggers = new ArrayList<>();

    private static double now, deltaTime;
    private static long prevTime;
    private static boolean started;

    private CommandScheduler() {}

    /// Resets whenever the scheduler has been reset.
    public static double now() {
        return now;
    }

    /// In seconds
    public static double getDeltaTime() {
        return deltaTime;
    }

    public static void reset() {

        for (Command command : running) command.stop(true);

        running.clear();
        pending.clear();
        cancelling.clear();
        subsystems.clear();
        triggers.clear();

        now = 0;
        deltaTime = 0;
        started = false;
    }

    public static void register(Subsystem... toRegister) {

        for (Subsystem subsystem : toRegister) {

            if (subsystems.containsKey(subsystem)) continue;

            subsystems.put(subsystem, subsystem.getDefaultCommand());
            subsystem.initialize();
        }
    }

    public static void setDefaultCommand(Subsystem subsystem, Command fallback) {

        register(subsystem);
        subsystems.put(subsystem, fallback);
    }

    public static <T extends Subsystem> T getSubsystem(Class<T> type) {

        for (Subsystem subsystem : subsystems.keySet()) {
            if (type.isInstance(subsystem)) return type.cast(subsystem);
        }

        throw new IllegalStateException("No " + type.getSimpleName() + " is registered with the scheduler!");
    }

    public static ArrayList<Subsystem> getSubsystems() {
        return new ArrayList<>(subsystems.keySet());
    }

    public static void schedule(Command... toSchedule) {

        for (Command command : toSchedule) {

            if (command == null || pending.contains(command) || running.contains(command)) continue;

            if (command.isComposed()) throw new IllegalArgumentException("This command is already part of a group, schedule the group instead!");

            register(command.getRequirements().toArray(new Subsystem[0]));

            pending.add(command);
        }
    }

    public static void cancel(Command... toCancel) {

        for (Command command : toCancel) {

            pending.remove(command);

            if (running.contains(command) && !cancelling.contains(command)) cancelling.add(command);
        }
    }

    public static void cancelAll() {

        pending.clear();

        for (Command command : running) if (!cancelling.contains(command)) cancelling.add(command);

        sweep();
    }

    public static boolean isScheduled(Command command) {
        return running.contains(command) || pending.contains(command);
    }

    public static ArrayList<Command> getRunning() {
        return new ArrayList<>(running);
    }

    /// Must be called every loop.
    public static void run() {

        tick();

        for (Trigger trigger : new ArrayList<>(triggers)) {
            trigger.poll();
        }

        for (Subsystem subsystem : new ArrayList<>(subsystems.keySet())) {
            subsystem.loop();
        }

        sweep();
        admit();

        for (Command command : new ArrayList<>(running)) {

            if (cancelling.contains(command)) continue;

            if (command.shouldCancel()) {

                cancelling.add(command);
                continue;
            }

            command.update();

            if (command.isDone()) {

                command.stop(false);
                running.remove(command);
            }
        }

        sweep();
        fillIdleSubsystems();
    }

    public static void idle() {

        tick();

        for (Subsystem subsystem : new ArrayList<>(subsystems.keySet())) {
            subsystem.loop();
        }
    }

    static void register(Trigger trigger) {
        triggers.add(trigger);
    }

    private static void tick() {

        long currTime = System.nanoTime();

        if (!started) {

            started = true;
            prevTime = currTime;
        }

        deltaTime = (currTime - prevTime) / 1e9;
        now += deltaTime;
        prevTime = currTime;
    }

    private static void admit() {

        for (Command command : new ArrayList<>(pending)) {

            pending.remove(command);

            ArrayList<Command> displaced = new ArrayList<>();
            boolean blocked = false;

            for (Command other : running) {

                if (!shares(command, other)) continue;

                if (!other.isInterruptible()) {
                    blocked = true;
                    break;
                }

                displaced.add(other);
            }

            if (blocked) continue;

            for (Command other : displaced) {

                other.stop(true);
                running.remove(other);
                cancelling.remove(other);
            }

            running.add(command);
            command.begin();
        }
    }

    private static void sweep() {

        for (Command command : new ArrayList<>(cancelling)) {

            if (running.remove(command)) command.stop(true);

            cancelling.remove(command);
        }
    }

    private static void fillIdleSubsystems() {

        for (Subsystem subsystem : new ArrayList<>(subsystems.keySet())) {

            Command fallback = subsystems.get(subsystem);

            if (fallback == null || isScheduled(fallback)) continue;

            boolean busy = false;

            for (Command command : running) {
                if (command.getRequirements().contains(subsystem)) { busy = true; break; }
            }

            if (!busy) schedule(fallback);
        }
    }

    private static boolean shares(Command a, Command b) {

        for (Subsystem subsystem : a.getRequirements()) {
            if (b.getRequirements().contains(subsystem)) return true;
        }

        return false;
    }
}
