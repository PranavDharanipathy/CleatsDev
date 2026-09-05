package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.PoseLQRTuner;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class TranslationLQRTest extends LinearOpMode {

    // In case you may want a different value, do not go below 3 or above 10.
    public static int LOOP_ITERATIONS_PER_TIME_CONSTANT = 10;

    public static double TEST_DISTANCE = 12; //inches

    private static final double ALREADY_CLOSE_THRESHOLD_POSITION = 1; //inches
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

        double awayX = start.x + (TEST_DISTANCE / Math.sqrt(2d)) * (Math.cos(start.heading) + Math.sin(start.heading));
        double awayY = start.y + (TEST_DISTANCE / Math.sqrt(2d)) * (Math.sin(start.heading) - Math.cos(start.heading));
        Pose away = new Pose(awayX, awayY, start.heading);

        PoseLQRTuner tuner = new PoseLQRTuner(pc.getMotionConstraints(), LOOP_ITERATIONS_PER_TIME_CONSTANT);
        MecanumProfile profile = pc.getMecanumProfile();

        boolean stopRequested = false;

        while (opModeIsActive()) {

            if (gamepad1.a) stopRequested = true;

            //driving away
            pc.follow(new HermiteSpline(pc.getFinalLocalizer().getPose(), away)
                    .setHeadingOp(HeadingOp.constantHeading(start.heading)), false);

            while (opModeIsActive() && pc.isFollowing()) {

                if (gamepad1.a) stopRequested = true;

                pc.update();
                telemetry.addLine("Moving away with transit mode");
                telemetry.update();
            }

            //driving back (using LQR)
            Double initialForwardError = null, initialStrafeError = null;
            boolean forwardOvershot = false, strafeOvershot = false;
            double overshootMagnitude = 0;
            double legStartTime = getRuntime();

            while (opModeIsActive()) {

                if (gamepad1.a) stopRequested = true;

                pc.update();
                Pose pose = pc.getFinalLocalizer().getPose();
                Pose velocity = pc.getFinalLocalizer().getVelocity();
                double dt = pc.getFinalLocalizer().getDeltaTime();

                double fieldErrorX = pose.x - start.x;
                double fieldErrorY = pose.y - start.y;

                double forwardError = fieldErrorX * Math.cos(pose.heading) + fieldErrorY * Math.sin(pose.heading);
                double strafeError = fieldErrorX * Math.sin(pose.heading) - fieldErrorY * Math.cos(pose.heading);

                double forwardVelocity = velocity.x * Math.cos(pose.heading) + velocity.y * Math.sin(pose.heading);
                double strafeVelocity = velocity.x * Math.sin(pose.heading) - velocity.y * Math.cos(pose.heading);

                double distance = Math.hypot(fieldErrorX, fieldErrorY);
                double speed = Math.hypot(velocity.x, velocity.y);

                if (initialForwardError == null) {
                    initialForwardError = forwardError;
                    initialStrafeError = strafeError;
                }

                if (initialForwardError != 0 && Math.signum(forwardError) != Math.signum(initialForwardError)) {
                    forwardOvershot = true;
                    overshootMagnitude = Math.max(overshootMagnitude, Math.abs(forwardError));
                }
                if (initialStrafeError != 0 && Math.signum(strafeError) != Math.signum(initialStrafeError)) {
                    strafeOvershot = true;
                    overshootMagnitude = Math.max(overshootMagnitude, Math.abs(strafeError));
                }

                PoseLQRController lqr = tuner.update(forwardError, forwardVelocity, strafeError, strafeVelocity, 0, 0, dt);

                double forwardCorrection = lqr.correctForward(forwardError, forwardVelocity);
                double strafeCorrection = lqr.correctStrafe(strafeError, strafeVelocity);

                double forwardPower = MathHelper.clamp(forwardCorrection / profile.getMaxAcceleration(0), -1, 1);
                double strafePower = MathHelper.clamp(strafeCorrection / profile.getMaxAcceleration(Math.PI / 2d), -1, 1);

                pc.getChassis().setDrivePower(forwardPower, strafePower, 0, dt);

                //debug info
                telemetry.addData("distance (in)", distance);
                telemetry.addData("speed (in/s)", speed);
                telemetry.addData("forward power", forwardPower);
                telemetry.addData("strafe power", strafePower);
                telemetry.addData("saturated", Math.abs(forwardPower) >= 1 || Math.abs(strafePower) >= 1);
                telemetry.addData("overshoot detected", forwardOvershot || strafeOvershot);
                telemetry.addData("overshoot magnitude (in)", overshootMagnitude);

                boolean timedOut = getRuntime() - legStartTime > MAX_RETURN_TIME;
                telemetry.addData("did robot timeout", timedOut);
                telemetry.update();

                if (distance < ALREADY_CLOSE_THRESHOLD_POSITION || timedOut) break;
            }

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

            if (stopRequested) break;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== LQR PARAMETERS ===");
        telemetry.addData("qPositionForward", tuner.getLastQPositionForward());
        telemetry.addData("qVelocityForward", tuner.getLastQVelocityForward());
        telemetry.addData("qPositionStrafe", tuner.getLastQPositionStrafe());
        telemetry.addData("qVelocityStrafe", tuner.getLastQVelocityStrafe());
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
