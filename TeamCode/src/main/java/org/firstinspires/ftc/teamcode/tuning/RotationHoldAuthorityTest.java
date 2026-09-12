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

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("Clear about 6ft square. The robot turns in place and shoves itself sideways mid turn, then drives back to where it started.");
        telemetry.update();

        waitForStart();

        pc.update();

        Pose home = pc.getPose().copy();

        double[] meanHeading = new double[SHARES.length];
        double[] worstHeading = new double[SHARES.length];
        double[] meanLeftOff = new double[SHARES.length];
        double[] meanPushed = new double[SHARES.length];

        for (int s = 0; s < SHARES.length && opModeIsActive(); s++) {

            pc.setRotationHoldAuthority(SHARES[s]);

            for (int r = 0; r < REPEATS && opModeIsActive(); r++) {

                double shoveAt = FIRST_SHOVE_AT + r * SHOVE_SPACING;
                double direction = (r % 2 == 0) ? 1 : -1;

                double[] result = trial(TURN_DEGREES * direction, shoveAt);

                meanHeading[s] += result[0] / REPEATS;
                worstHeading[s] = Math.max(worstHeading[s], result[0]);
                meanLeftOff[s] += result[1] / REPEATS;
                meanPushed[s] += result[2] / REPEATS;

                telemetry.addLine("=== TRIAL ===");
                telemetry.addData("share", SHARES[s]);
                telemetry.addData("repeat", (r + 1) + " of " + REPEATS);
                telemetry.addData("heading left over (deg)", result[0]);
                telemetry.addData("left off its spot (in)", result[1]);
                telemetry.addData("pushed off by (in)", result[2]);
                telemetry.update();

                returnTo(home);
            }
        }

        report(meanHeading, worstHeading, meanLeftOff, meanPushed);

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

            //straight after the controller, so this loop's command is the shove instead
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

    private void returnTo(Pose home) {

        pc.update();

        if (Math.hypot(pc.getX() - home.x, pc.getY() - home.y) < 2) return;

        pc.follow(new HermiteSpline(pc.getPose().copy(), home.copy()));

        double started = getRuntime();

        while (opModeIsActive() && pc.isFollowing() && getRuntime() - started < RETURN_TIMEOUT) {

            pc.update();

            telemetry.addLine("driving back to the start spot");
            telemetry.addData("away (in)", Math.hypot(pc.getX() - home.x, pc.getY() - home.y));
            telemetry.update();
        }

        pc.cancel();
    }

    private void report(double[] meanHeading, double[] worstHeading, double[] meanLeftOff, double[] meanPushed) {

        double bestHold = Double.MAX_VALUE;
        for (double held : meanLeftOff) bestHold = Math.min(bestHold, held);

        double allowedHeading = Math.toDegrees(pc.getBrakingModel().getAngularMargin());

        //the smallest share that already holds position as well as any larger one does
        //that doesn't induce more slack
        double pick = 0;

        for (int i = 0; i < SHARES.length; i++) {

            if (meanLeftOff[i] > bestHold * 1.05) continue;
            if (worstHeading[i] > allowedHeading) continue;

            pick = SHARES[i];
            break;
        }

        telemetry.addLine("=== ROTATION HOLD AUTHORITY ===");
        telemetry.addData("turn's own angular margin (deg)", allowedHeading);
        telemetry.addData("ROTATION_HOLD_AUTHORITY", pick);
        telemetry.update();
    }
}
