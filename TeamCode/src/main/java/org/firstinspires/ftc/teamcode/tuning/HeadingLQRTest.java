package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.PoseLQRTuner;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class HeadingLQRTest extends LinearOpMode {

    // In case you may want a different value, do not go below 3 or above 10.
    public static int LOOP_ITERATIONS_PER_TIME_CONSTANT = 10;

    public static double TEST_ANGLE_DEGREES = 30;

    private static final double ALREADY_CLOSE_THRESHOLD_HEADING = 1; //radians
    private static final double MAX_RETURN_TIME = 5;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController();

        telemetry.addLine("Robot will oscillate under self-tuned LQR the entire time.");
        telemetry.addLine("Press A to end, it'll finishes the current path before stopping.");
        telemetry.update();

        waitForStart();

        pc.update();
        Pose start = pc.getFinalLocalizer().getPose();
        double startHeading = start.heading;
        double awayHeading = startHeading + Math.toRadians(TEST_ANGLE_DEGREES);

        PoseLQRTuner tuner = new PoseLQRTuner(pc.getMotionConstraints(), LOOP_ITERATIONS_PER_TIME_CONSTANT);
        double amaxH = pc.getMotionConstraints().getAmaxH();

        boolean stopRequested = false;

        while (opModeIsActive()) {

            if (gamepad1.a) stopRequested = true;

            //turn away
            Pose currentPose = pc.getFinalLocalizer().getPose();
            Pose away = new Pose(currentPose.x, currentPose.y, awayHeading);
            pc.follow(new HermiteSpline(currentPose, away)
                    .setHeadingOp(HeadingOp.linearHeading(currentPose.heading, awayHeading)), false);

            while (opModeIsActive() && pc.isFollowing()) {

                if (gamepad1.a) stopRequested = true;

                pc.update();
                telemetry.addLine("turning away with transit mode");
                telemetry.update();
            }

            //turning back (using LQR)
            Double initialHeadingError = null;
            boolean overshotDetected = false;
            double overshootMagnitude = 0;
            double legStartTime = getRuntime();

            while (opModeIsActive()) {

                if (gamepad1.a) stopRequested = true;

                pc.update();
                Pose pose = pc.getFinalLocalizer().getPose();
                Pose velocity = pc.getFinalLocalizer().getVelocity();
                double dt = pc.getFinalLocalizer().getDeltaTime();

                double headingError = MathHelper.normalizeAngleRad(pose.heading - startHeading);
                double absError = Math.abs(headingError);
                double angularSpeed = Math.abs(velocity.heading);

                if (initialHeadingError == null) {
                    initialHeadingError = headingError;
                }

                if (initialHeadingError != 0 && Math.signum(headingError) != Math.signum(initialHeadingError)) {
                    overshotDetected = true;
                    overshootMagnitude = Math.max(overshootMagnitude, absError);
                }

                PoseLQRController lqr = tuner.update(0, 0, 0, 0, absError, angularSpeed, dt);

                double correction = lqr.correctHeading(headingError, velocity.heading);
                double turnPower = MathHelper.clamp(correction / amaxH, -1, 1);

                pc.getChassis().setDrivePower(0, 0, turnPower, dt);

                //debug info
                telemetry.addData("heading error (deg)", Math.toDegrees(headingError));
                telemetry.addData("angular velocity (rad/s)", velocity.heading);
                telemetry.addData("turn power", turnPower);
                telemetry.addData("saturated", Math.abs(turnPower) >= 1);
                telemetry.addData("overshoot detected", overshotDetected);
                telemetry.addData("overshoot magnitude (deg)", Math.toDegrees(overshootMagnitude));

                boolean timedOut = getRuntime() - legStartTime > MAX_RETURN_TIME;
                telemetry.addData("did robot timeout", timedOut);
                telemetry.update();

                if (absError < ALREADY_CLOSE_THRESHOLD_HEADING || timedOut) break;
            }

            pc.getChassis().setDrivePower(0, 0, 0, pc.getFinalLocalizer().getDeltaTime());

            if (stopRequested) break;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== LQR PARAMETERS ===");
        telemetry.addData("qPositionHeading", tuner.getLastQPositionHeading());
        telemetry.addData("qVelocityHeading", tuner.getLastQVelocityHeading());
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
