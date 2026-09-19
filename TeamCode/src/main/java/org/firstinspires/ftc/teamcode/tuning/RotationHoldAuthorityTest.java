package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Turn;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class RotationHoldAuthorityTest extends LinearOpMode {

    public static double TURN_DEGREES = 180;
    public static int REPEATS = 6;

    public static double SHOVE_POWER = 0.8;
    public static double SHOVE_SECONDS = 0.4;
    public static double FIRST_SHOVE_AT = 0.2;
    public static double SHOVE_SPACING = 0.05;

    public static double TRIAL_TIMEOUT = 8;
    public static double RETURN_TIMEOUT = 8;

    private static final double[] SHARES = {0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.45, 0.7, 1};

    private PathController pc;
    private Pose home;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("Clear about 6ft square. The robot turns in place and shoves itself sideways mid turn, then drives back to where it started.");
        telemetry.addLine("B at any time to drive the robot somewhere clearer.");
        telemetry.update();

        waitForStart();

        pc.update();

        home = pc.getPose().copy();

        double[] worstHeading = new double[SHARES.length];
        double[] meanLeftOff = new double[SHARES.length];
        double[] meanPushed = new double[SHARES.length];

        //trials that were cut short cannot be used
        int[] trials = new int[SHARES.length];

        for (int s = 0; s < SHARES.length && opModeIsActive(); s++) {

            pc.setRotationHoldAuthority(SHARES[s]);

            for (int r = 0; r < REPEATS && opModeIsActive(); r++) {

                double shoveAt = FIRST_SHOVE_AT + r * SHOVE_SPACING;
                double direction = (r % 2 == 0) ? 1 : -1;

                double[] result;

                //interrupted trials runs again
                do {
                    repositioned = false;
                    result = trial(TURN_DEGREES * direction, shoveAt);
                }
                while (repositioned && opModeIsActive());

                if (result == null) break;

                trials[s]++;

                worstHeading[s] = Math.max(worstHeading[s], result[0]);
                meanLeftOff[s] += result[1];
                meanPushed[s] += result[2];

                telemetry.addLine("=== TRIAL ===   (B to reposition)");
                telemetry.addData("share", SHARES[s]);
                telemetry.addData("repeat", (r + 1) + " of " + REPEATS);
                telemetry.addData("heading left over (deg)", result[0]);
                telemetry.addData("left off its spot (in)", result[1]);
                telemetry.addData("pushed off by (in)", result[2]);
                telemetry.update();

                returnTo();
            }

            if (trials[s] > 0) {

                meanLeftOff[s] /= trials[s];
                meanPushed[s] /= trials[s];
            }
        }

        report(worstHeading, meanLeftOff, meanPushed, trials);

        while (opModeIsActive()) ;
    }

    private double[] trial(double turnDegrees, double shoveAt) {

        pc.update();

        Pose startedOn = pc.getPose().copy();
        double wanted = MathHelper.normalizeAngleRad(startedOn.heading + Math.toRadians(turnDegrees));

        pc.follow(Turn.degrees(turnDegrees));

        double started = getRuntime();
        double pushed = 0;

        while (opModeIsActive() && pc.isFollowing()) {

            double t = getRuntime() - started;
            if (t > TRIAL_TIMEOUT) break;

            pc.update();

            if (reposition()) return null;

            if (t > shoveAt && t < shoveAt + SHOVE_SECONDS) pc.getChassis().setDrivePowerBypassRamp(0, SHOVE_POWER, 0);

            pushed = Math.max(pushed, Math.hypot(pc.getX() - startedOn.x, pc.getY() - startedOn.y));
        }

        pc.cancel();
        pc.update();

        return new double[]{
                Math.toDegrees(Math.abs(MathHelper.normalizeAngleRad(pc.getHeading() - wanted))),
                Math.hypot(pc.getX() - startedOn.x, pc.getY() - startedOn.y),
                pushed
        };
    }

    private void returnTo() {

        pc.update();

        if (Math.hypot(pc.getX() - home.x, pc.getY() - home.y) < 2) return;

        pc.follow(new HermiteSpline(pc.getPose().copy(), home.copy()));

        double started = getRuntime();

        while (opModeIsActive() && pc.isFollowing() && getRuntime() - started < RETURN_TIMEOUT) {

            pc.update();

            if (reposition()) return;

            telemetry.addLine("driving back to the start spot   (B to reposition)");
            telemetry.addData("away (in)", Math.hypot(pc.getX() - home.x, pc.getY() - home.y));
            telemetry.update();
        }

        pc.cancel();
    }

    private boolean repositioned;

    private boolean reposition() {

        if (!gamepad1.b) return false;

        pc.cancel();
        repositioned = true;

        while (opModeIsActive() && gamepad1.b) pc.update();

        while (opModeIsActive()) {

            pc.update();

            pc.getChassis().driveFromJoystick(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addLine("driving free, B when the robot is where you want it");
            telemetry.update();

            if (gamepad1.b) break;
        }

        while (opModeIsActive() && gamepad1.b) pc.update();

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        pc.update();
        home = pc.getPose().copy();

        return true;
    }

    private void report(double[] worstHeading, double[] meanLeftOff, double[] meanPushed, int[] trials) {

        double bestHold = Double.MAX_VALUE;

        for (int i = 0; i < SHARES.length; i++) {
            if (trials[i] > 0) bestHold = Math.min(bestHold, meanLeftOff[i]);
        }

        double allowedHeading = Math.toDegrees(pc.getBrakingModel().getAngularMargin());

        double pick = 0;
        boolean found = false;

        for (int i = 0; i < SHARES.length; i++) {

            if (trials[i] == 0) continue;
            if (meanLeftOff[i] > bestHold * 1.05) continue;
            if (worstHeading[i] > allowedHeading) continue;

            pick = SHARES[i];
            found = true;

            break;
        }

        telemetry.addLine("=== ROTATION HOLD AUTHORITY ===");
        telemetry.addData("turn's own angular margin (deg)", allowedHeading);

        if (!found) {

            telemetry.addLine("No share held its heading inside that margin, so there is nothing to pick.");
            telemetry.update();

            return;
        }

        telemetry.addData("ROTATION_HOLD_AUTHORITY", pick);

        //the shove has to actually knock the robot off-path
        if (trials[0] > 0 && meanLeftOff[0] <= bestHold * 1.05) {

            telemetry.addLine("The shove never knocked the robot off, so holding position made no difference here.");
            telemetry.addData("shove moved the robot (in)", meanPushed[0]);
        }

        telemetry.update();
    }
}
