package org.firstinspires.ftc.teamcode.dribble;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/// Provides the running OpMode's hardware devices, gamepads, telemetry, etc.
public final class DribbleUtility {

    public enum State {
        IDLE, INIT, STARTED, STOPPED
    }

    private static OpMode opMode;
    private static State state = State.IDLE;

    private DribbleUtility() {}

    /// Call once at the top of an OpMode. {@link DribbleOpMode} already does it.
    public static void setUp(OpMode opMode) {

        DribbleUtility.opMode = opMode;
        state = State.INIT;
    }

    public static void setState(State state) {
        DribbleUtility.state = state;
    }

    public static State getState() {

        //a plain LinearOpMode doesn't tell us that it started, so we read it off the OpMode instead
        if (state != State.IDLE && opMode instanceof LinearOpMode) {

            LinearOpMode linear = (LinearOpMode) opMode;

            if (linear.isStopRequested()) state = State.STOPPED;
            else if (state == State.INIT && linear.isStarted()) state = State.STARTED;
        }

        return state;
    }

    public static boolean isInit() {
        return getState() == State.INIT;
    }

    public static boolean isStarted() {
        return getState() == State.STARTED;
    }

    public static boolean isStopped() {
        return getState() == State.STOPPED;
    }

    public static OpMode getOpMode() {
        return require();
    }

    public static HardwareMap getHardwareMap() {
        return require().hardwareMap;
    }

    public static Telemetry getTelemetry() {
        return require().telemetry;
    }

    public static Gamepad getGamepad1() {
        return require().gamepad1;
    }

    public static Gamepad getGamepad2() {
        return require().gamepad2;
    }

    public static <T extends Subsystem> T getSubsystem(Class<T> type) {
        return CommandScheduler.getSubsystem(type);
    }

    private static OpMode require() {

        if (opMode == null) throw new IllegalStateException("No OpMode has been set up yet, call DribbleUtility.setUp(this) first!");

        return opMode;
    }
}
