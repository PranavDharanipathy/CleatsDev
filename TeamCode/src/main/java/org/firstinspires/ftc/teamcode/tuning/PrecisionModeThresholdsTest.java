package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.PoseLQRTuner;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class PrecisionModeThresholdsTest extends LinearOpMode {

    // PLEASE use the debugging data printed onto telemetry to help you
    // change these values in the event where that may be necessary.

    // In case you may want a different value, do not go below 3 or above 10.
    public static int LOOP_ITERATIONS_PER_TIME_CONSTANT = 10;

    public static double POSITION_SEARCH_MIN = 1; //inches
    public static double POSITION_SEARCH_MAX = 36; //inches
    public static double EXIT_POSITION_SEARCH_MAX = 72; //inches

    public static double HEADING_SEARCH_MIN_DEGREES = 1;
    public static double HEADING_SEARCH_MAX_DEGREES = 60; //entry search ceiling
    public static double EXIT_HEADING_SEARCH_MAX_DEGREES = 90; //exit search ceiling

    public static int SEARCH_ITERATIONS = 10;

    // Put here the value produced by LQROvershootDiagnosticTest
    private static final double ALREADY_CLOSE_THRESHOLD_POSITION = 1;
    private static final double ALREADY_CLOSE_THRESHOLD_HEADING = 1;

    private static final double MAX_TRIAL_TIME = 5;

    private PathController pc;
    private PoseLQRTuner tuner;
    private MecanumProfile profile;
    private double amaxH, dmaxH;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController();

        telemetry.addLine("Give it space (it will move diagonally forward-right and later also turn).");
        telemetry.addLine("Press B at any time to manually drive and reposition the robot, press B again to resume the search.");
        telemetry.update();

        waitForStart();

        pc.update();

        tuner = new PoseLQRTuner(pc.getMotionConstraints(), LOOP_ITERATIONS_PER_TIME_CONSTANT);
        profile = pc.getMecanumProfile();
        dmaxH = pc.getMotionConstraints().getDmaxH();
        amaxH = pc.getMotionConstraints().getAmaxH();

        double entryPositionDistance = searchEntryPosition();
        double exitPositionDistance = searchExitPosition(entryPositionDistance);

        double entryHeadingError = searchEntryHeading();
        double exitHeadingError = searchExitHeading(entryHeadingError);

        double maxDecelDiagonal = profile.getMaxDeceleration(Math.PI / 4d);
        double entryVelocity = Math.sqrt(2d * maxDecelDiagonal * entryPositionDistance);
        double exitVelocity = Math.sqrt(2d * maxDecelDiagonal * exitPositionDistance);

        double entryAngularVelocity = Math.sqrt(2d * dmaxH * entryHeadingError);
        double exitAngularVelocity = Math.sqrt(2d * dmaxH * exitHeadingError);

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== PrecisionModeThresholds ===");
        telemetry.addData("entryPositionDistance", entryPositionDistance);
        telemetry.addData("exitPositionDistance", exitPositionDistance);
        telemetry.addData("entryVelocity", entryVelocity);
        telemetry.addData("exitVelocity", exitVelocity);
        telemetry.addData("entryHeadingError", entryHeadingError);
        telemetry.addData("exitHeadingError", exitHeadingError);
        telemetry.addData("entryAngularVelocity", entryAngularVelocity);
        telemetry.addData("exitAngularVelocity", exitAngularVelocity);

        //telemetry.addData("exit/entry position ratio", exitPositionDistance / entryPositionDistance);
        //telemetry.addData("exit/entry heading ratio", exitHeadingError / entryHeadingError);

        if (entryPositionDistance > POSITION_SEARCH_MAX * 0.95) {
            telemetry.addLine("Entry position result is near the search ceiling, raise POSITION_SEARCH_HI and run again.");
        }
        if (exitPositionDistance > EXIT_POSITION_SEARCH_MAX * 0.95) {
            telemetry.addLine("Exit position result is near the search ceiling, raise EXIT_POSITION_SEARCH_HI and run again.");
        }
        if (entryHeadingError > Math.toRadians(HEADING_SEARCH_MAX_DEGREES) * 0.95) {
            telemetry.addLine("Entry heading result is near the search ceiling, raise HEADING_SEARCH_HI_DEGREES and run again.");
        }
        if (exitHeadingError > Math.toRadians(EXIT_HEADING_SEARCH_MAX_DEGREES) * 0.95) {
            telemetry.addLine("Exit heading result is near the search ceiling, raise EXIT_HEADING_SEARCH_HI_DEGREES and run again.");
        }

        telemetry.update();

        while (opModeIsActive()) ;
    }

    private boolean bWasPressed = false;

    private void checkManualDriving() {

        boolean pressed = gamepad1.b;
        boolean justPressed = pressed && !bWasPressed;
        bWasPressed = pressed;

        if (!justPressed) return;

        boolean exitEdge = true; //b is currently held down from the entry press

        while (opModeIsActive()) {

            pc.update();
            double dt = pc.getFinalLocalizer().getDeltaTime();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            pc.getChassis().setDrivePower(forward, strafe, turn, dt);

            telemetry.addLine("MANUAL DRIVING, press B to resume search");
            telemetry.update();

            boolean nowPressed = gamepad1.b;
            if (nowPressed && !exitEdge) break;
            exitEdge = nowPressed;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        bWasPressed = gamepad1.b;
    }

    //searches utilize binary search

    private double searchEntryPosition() {

        double lo = POSITION_SEARCH_MIN;
        double hi = POSITION_SEARCH_MAX;

        for (int i = 0; i < SEARCH_ITERATIONS && opModeIsActive(); i++) {

            double mid = (lo + hi) / 2d;
            TrialResult result = runPositionTrial(mid);

            if (result.clean) lo = mid;
            else hi = mid;
        }

        return lo;
    }

    private double searchExitPosition(double entryPositionDistance) {

        double lo = entryPositionDistance; //exit can't be smaller than entry
        double hi = EXIT_POSITION_SEARCH_MAX;

        for (int i = 0; i < SEARCH_ITERATIONS && opModeIsActive(); i++) {

            double mid = (lo + hi) / 2d;
            TrialResult result = runPositionTrial(mid);

            if (result.converged) lo = mid;
            else hi = mid;
        }

        return lo;
    }

    private double searchEntryHeading() {

        double lo = Math.toRadians(HEADING_SEARCH_MIN_DEGREES);
        double hi = Math.toRadians(HEADING_SEARCH_MAX_DEGREES);

        for (int i = 0; i < SEARCH_ITERATIONS && opModeIsActive(); i++) {

            double mid = (lo + hi) / 2d;
            TrialResult result = runHeadingTrial(mid);

            if (result.clean) lo = mid;
            else hi = mid;
        }

        return lo;
    }

    private double searchExitHeading(double entryHeadingError) {

        double lo = entryHeadingError; //exit can't be smaller than entry
        double hi = Math.toRadians(EXIT_HEADING_SEARCH_MAX_DEGREES);

        for (int i = 0; i < SEARCH_ITERATIONS && opModeIsActive(); i++) {

            double mid = (lo + hi) / 2d;
            TrialResult result = runHeadingTrial(mid);

            if (result.converged) lo = mid;
            else hi = mid;
        }

        return lo;
    }

    private static class TrialResult { // <- classmaxxing is tuff
        boolean clean; //if there's no overshoot/saturation
        boolean converged; //if reached ALREADY_CLOSE_THRESHOLD before MAX_TRIAL_TIME
    }

    private TrialResult runPositionTrial(double distance) {

        Pose localStart = pc.getFinalLocalizer().getPose();

        double awayX = localStart.x + (distance / Math.sqrt(2d)) * (Math.cos(localStart.heading) + Math.sin(localStart.heading));
        double awayY = localStart.y + (distance / Math.sqrt(2d)) * (Math.sin(localStart.heading) - Math.cos(localStart.heading));
        Pose away = new Pose(awayX, awayY, localStart.heading);

        pc.follow(new HermiteSpline(localStart, away), false);

        while (opModeIsActive() && pc.isFollowing()) {

            checkManualDriving();

            pc.update();
            telemetry.addLine("POSITION TRIAL");
            telemetry.addData("candidate (in)", distance);
            telemetry.update();
        }

        boolean saturated = false, overshot = false;
        Double initialForwardError = null, initialStrafeError = null;
        double legStartTime = getRuntime();
        boolean converged = false;

        while (opModeIsActive()) {

            checkManualDriving();

            pc.update();
            Pose pose = pc.getFinalLocalizer().getPose();
            Pose velocity = pc.getFinalLocalizer().getVelocity();
            double dt = pc.getFinalLocalizer().getDeltaTime();

            double fieldErrorX = pose.x - localStart.x;
            double fieldErrorY = pose.y - localStart.y;

            double forwardError = fieldErrorX * Math.cos(pose.heading) + fieldErrorY * Math.sin(pose.heading);
            double strafeError = fieldErrorX * Math.sin(pose.heading) - fieldErrorY * Math.cos(pose.heading);

            double forwardVelocity = velocity.x * Math.cos(pose.heading) + velocity.y * Math.sin(pose.heading);
            double strafeVelocity = velocity.x * Math.sin(pose.heading) - velocity.y * Math.cos(pose.heading);

            double distanceNow = Math.hypot(fieldErrorX, fieldErrorY);

            if (initialForwardError == null) {
                initialForwardError = forwardError;
                initialStrafeError = strafeError;
            }
            if (initialForwardError != 0 && Math.signum(forwardError) != Math.signum(initialForwardError)) overshot = true;
            if (initialStrafeError != 0 && Math.signum(strafeError) != Math.signum(initialStrafeError)) overshot = true;

            PoseLQRController lqr = tuner.update(forwardError, forwardVelocity, strafeError, strafeVelocity, 0, 0, dt);

            double forwardCorrection = lqr.correctForward(forwardError, forwardVelocity);
            double strafeCorrection = lqr.correctStrafe(strafeError, strafeVelocity);

            double forwardPower = MathHelper.clamp(forwardCorrection / profile.getMaxAcceleration(0), -1, 1);
            double strafePower = MathHelper.clamp(strafeCorrection / profile.getMaxAcceleration(Math.PI / 2d), -1, 1);

            if (Math.abs(forwardPower) >= 1 || Math.abs(strafePower) >= 1) saturated = true;

            pc.getChassis().setDrivePower(forwardPower, strafePower, 0, dt);

            telemetry.addLine("POSITION TRIAL");
            telemetry.addData("candidate (in)", distance);
            telemetry.addData("current distance (in)", distanceNow);
            telemetry.addData("saturated this trial", saturated);
            telemetry.addData("overshot this trial", overshot);
            telemetry.update();

            boolean timedOut = getRuntime() - legStartTime > MAX_TRIAL_TIME;

            if (distanceNow < ALREADY_CLOSE_THRESHOLD_POSITION) {
                converged = true;
                break;
            }
            if (timedOut) break;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        TrialResult result = new TrialResult();
        result.clean = !saturated && !overshot;
        result.converged = converged;
        return result;
    }

    private TrialResult runHeadingTrial(double angle) {

        Pose localStart = pc.getFinalLocalizer().getPose();
        double startHeading = localStart.heading;
        Pose away = new Pose(localStart.x, localStart.y, startHeading + angle);

        pc.follow(new HermiteSpline(localStart, away), false);

        while (opModeIsActive() && pc.isFollowing()) {

            checkManualDriving();

            pc.update();
            telemetry.addLine("HEADING TRIAL");
            telemetry.addData("candidate (deg)", Math.toDegrees(angle));
            telemetry.update();
        }

        boolean saturated = false, overshot = false;
        Double initialHeadingError = null;
        double legStartTime = getRuntime();
        boolean converged = false;

        while (opModeIsActive()) {

            checkManualDriving();

            pc.update();
            Pose pose = pc.getFinalLocalizer().getPose();
            Pose velocity = pc.getFinalLocalizer().getVelocity();
            double dt = pc.getFinalLocalizer().getDeltaTime();

            double headingError = MathHelper.normalizeAngleRad(pose.heading - startHeading);
            double absError = Math.abs(headingError);
            double angularSpeed = Math.abs(velocity.heading);

            if (initialHeadingError == null) initialHeadingError = headingError;
            if (initialHeadingError != 0 && Math.signum(headingError) != Math.signum(initialHeadingError)) overshot = true;

            PoseLQRController lqr = tuner.update(0, 0, 0, 0, absError, angularSpeed, dt);

            double correction = lqr.correctHeading(headingError, velocity.heading);
            double turnPower = MathHelper.clamp(correction / amaxH, -1, 1);

            if (Math.abs(turnPower) >= 1) saturated = true;

            pc.getChassis().setDrivePower(0, 0, turnPower, dt);

            telemetry.addLine("HEADING TRIAL");
            telemetry.addData("candidate (deg)", Math.toDegrees(angle));
            telemetry.addData("current error (deg)", Math.toDegrees(absError));
            telemetry.addData("saturated this trial", saturated);
            telemetry.addData("overshot this trial", overshot);
            telemetry.update();

            boolean timedOut = getRuntime() - legStartTime > MAX_TRIAL_TIME;

            if (absError < ALREADY_CLOSE_THRESHOLD_HEADING) {
                converged = true;
                break;
            }
            if (timedOut) break;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        TrialResult result = new TrialResult();
        result.clean = !saturated && !overshot;
        result.converged = converged;
        return result;
    }
}
