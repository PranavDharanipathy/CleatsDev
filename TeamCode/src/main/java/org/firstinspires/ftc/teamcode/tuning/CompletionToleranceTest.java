package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Maneuver;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.path.TurnTo;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class CompletionToleranceTest extends LinearOpMode {

    public static double LEG_DISTANCE = 24; //inches
    public static double TURN_DEGREES = 90;

    public static double SETTLE_SECONDS = 1;

    public static double HEADROOM = 1.5;

    private static final double MAX_LEG_TIME = 15;

    private PathController pc;
    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Clear a square area of about 48 inches.");
        telemetry.addLine("B at any time to drive the robot somewhere clearer.");
        telemetry.update();

        waitForStart();

        double position = 0, heading = 0;
        boolean measured = false;

        //moves in square shape
        double[][] legs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

        for (double[] leg : legs) {

            double[] band = retry("driving " + (int) LEG_DISTANCE + " in", () -> straightLeg(leg[0], leg[1]));

            if (band == null) continue;

            measured = true;
            position = Math.max(position, band[0]);
            heading = Math.max(heading, band[1]);
        }

        double[] turn = retry("turning " + (int) TURN_DEGREES + " deg", this::turnLeg);

        if (turn != null) {

            measured = true;
            heading = Math.max(heading, turn[1]);
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        if (!measured) {

            telemetry.addLine("Never reached precision mode on any leg.");
            telemetry.addLine("Tune the LQR with TranslationLQRTest and HeadingLQRTest first.");
            telemetry.update();

            while (opModeIsActive()) ;
            return;
        }

        telemetry.addLine("=== COMPLETION TOLERANCE RESULTS ===");
        telemetry.addData("COMPLETION_POSITION_EPSILON", "%.9f", position * HEADROOM);
        telemetry.addData("COMPLETION_HEADING_EPSILON", "Math.toRadians(%.9f)", Math.toDegrees(heading) * HEADROOM);

        telemetry.addLine();

        telemetry.addData("closest", "%.4f in, %.4f deg", position, Math.toDegrees(heading));
        telemetry.update();

        while (opModeIsActive()) ;
    }

    private interface Leg {
        double[] run();
    }

    //interrupted leg runs again
    private double[] retry(String label, Leg leg) {

        while (opModeIsActive()) {

            telemetry.addLine(label + "   (B to reposition)");
            telemetry.update();

            repositioned = false;

            double[] band = leg.run();

            if (!repositioned) return band;
        }

        return null;
    }

    private double[] straightLeg(double alongX, double alongY) {

        Pose from = pc.getPose();

        Pose to = new Pose(from.x + alongX * LEG_DISTANCE, from.y + alongY * LEG_DISTANCE);

        return hold(new Maneuver()
                .addMovement(new HermiteSpline(new Pose(from.x, from.y), to), null, true)
                .waitSeconds(MAX_LEG_TIME + SETTLE_SECONDS));
    }

    private double[] turnLeg() {

        return hold(new Maneuver()
                .addMovement(new TurnTo(pc.getHeading() + Math.toRadians(TURN_DEGREES)))
                .waitSeconds(MAX_LEG_TIME + SETTLE_SECONDS));
    }

    private double[] hold(Maneuver maneuver) {

        pc.follow(maneuver, true);

        double start = getRuntime();

        while (opModeIsActive() && !pc.isOnPrecisionMode() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            if (reposition()) return null;
        }

        if (!pc.isOnPrecisionMode()) {

            pc.cancel();
            return null;
        }

        Movement movement = pc.getCurrentMovement();

        if (movement == null) {

            pc.cancel();
            return null;
        }

        Pose held = movement.getEndPose();

        double until = getRuntime() + SETTLE_SECONDS;

        double position = Double.MAX_VALUE, heading = Double.MAX_VALUE;

        while (opModeIsActive() && getRuntime() < until && pc.isFollowing()) {

            pc.update();

            if (reposition()) return null;

            position = Math.min(position, Math.hypot(pc.getX() - held.x, pc.getY() - held.y));
            heading = Math.min(heading, Math.abs(MathHelper.normalizeAngleRad(pc.getHeading() - held.heading)));
        }

        boolean truncated = getRuntime() < until;

        pc.cancel();

        if (position == Double.MAX_VALUE || truncated) return null;

        return new double[] {position, heading};
    }

    private boolean repositioned;

    private boolean reposition() {

        if (!gamepad1.b) return false;

        pc.cancel();
        repositioned = true;

        while (opModeIsActive() && gamepad1.b) pc.update();

        while (opModeIsActive()) {

            pc.update();

            pc.getChassis().driveFromJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addLine("driving free, B when the robot is where you want it");
            telemetry.update();

            if (gamepad1.b) break;
        }

        while (opModeIsActive() && gamepad1.b) pc.update();

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        return true;
    }
}
