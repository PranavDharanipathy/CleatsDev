package org.firstinspires.ftc.teamcode.dribble;

public interface Subsystem {

    default void loop() {}

    default void initialize() {}

    /// What this subsystem does when nothing else has claimed it (null for nothing).
    default Command getDefaultCommand() {
        return null;
    }

    default <T extends Subsystem> T getSubsystem(Class<T> type) {
        return CommandScheduler.getSubsystem(type);
    }
}
