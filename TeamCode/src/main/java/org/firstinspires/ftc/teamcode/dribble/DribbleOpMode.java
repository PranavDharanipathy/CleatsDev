package org.firstinspires.ftc.teamcode.dribble;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class DribbleOpMode extends LinearOpMode {

    public void initialization() {}

    public void waitingForStart() {}

    public void onStart() {}

    public void onLoop() {}

    public void onStop() {}

    /// Registers subsystems with the scheduler
    public void register(Subsystem... subsystems) {
        CommandScheduler.register(subsystems);
    }

    /// Provides commands to the scheduler
    public void schedule(Command... commands) {
        CommandScheduler.schedule(commands);
    }

    @Override
    public final void runOpMode() {

        DribbleUtility.setUp(this);
        CommandScheduler.reset();

        initialization();

        while (!isStarted() && !isStopRequested()) {

            CommandScheduler.idle();

            waitingForStart();
            telemetry.update();
        }

        if (!isStopRequested()) {

            DribbleUtility.setState(DribbleUtility.State.STARTED);

            onStart();

            while (opModeIsActive()) {

                CommandScheduler.run();

                onLoop();
                telemetry.update();
            }
        }

        CommandScheduler.cancelAll();

        DribbleUtility.setState(DribbleUtility.State.STOPPED);

        onStop();
    }
}
