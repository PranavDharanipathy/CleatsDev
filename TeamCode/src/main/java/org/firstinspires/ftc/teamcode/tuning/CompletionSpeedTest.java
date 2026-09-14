package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class CompletionSpeedTest extends LinearOpMode {

    //six is where the fit stops improving
    public static int SAMPLES = 6;

    // What CompletionToleranceTest gave for COMPLETION_POSITION_EPSILON.
    public static double POSITION_TOLERANCE = 1;

    // What CompletionToleranceTest gave for COMPLETION_HEADING_EPSILON, in degrees.
    public static double HEADING_TOLERANCE_DEGREES = 2;

    private static final double MINIMUM_APPROACH_POWER = 0.3;

    private static final double PROBE_LOW_SHARE = 0.12;
    private static final double PROBE_HIGH_SHARE = 0.25;

    private static final double PROBE_RANGE = 2.5;

    private static final double MIN_RANGE_SHARE = 0.1;
    private static final double MAX_RANGE_SHARE = 0.6;

    private static final int MAX_CORRECTIONS = 4;
    private static final double STOPPED_SPEED = 0.5;
    private static final double STOPPED_ANGULAR_SPEED = 0.05;
    private static final double MAX_LEG_TIME = 8;

    private PathController pc;
    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Needs about a 48 inch square of clear floor.");
        telemetry.addLine("B at any time to drive the robot somewhere clearer.");
        telemetry.update();

        waitForStart();

        double headingTolerance = Math.toRadians(HEADING_TOLERANCE_DEGREES);

        double topSpeed = pc.getMotionConstraints().getVmaxF();
        double topTurn = pc.getMotionConstraints().getVmaxH();

        double maxTestSpeed = probeRange(topSpeed, false, POSITION_TOLERANCE);
        double maxTestTurn = probeRange(topTurn, true, headingTolerance);

        double[] forwardSpeeds = new double[SAMPLES], forward = new double[SAMPLES];
        double[] strafeSpeeds = new double[SAMPLES], strafe = new double[SAMPLES];
        double[] angularSpeeds = new double[SAMPLES], angular = new double[SAMPLES];

        for (int i = 0; i < SAMPLES && opModeIsActive(); i++) {

            double speed = maxTestSpeed * (i + 1) / SAMPLES;
            double angularSpeed = maxTestTurn * (i + 1) / SAMPLES;

            double[] leg = retry("forward coast from " + format(speed, 1), () -> coastLeg(speed, 1, 0));
            forwardSpeeds[i] = leg[0];
            forward[i] = leg[1];

            leg = retry("strafe coast from " + format(speed, 1), () -> coastLeg(speed, 0, 1));
            strafeSpeeds[i] = leg[0];
            strafe[i] = leg[1];

            leg = retry("turn coast from " + format(angularSpeed, 2), () -> angularCoastLeg(angularSpeed));
            angularSpeeds[i] = leg[0];
            angular[i] = leg[1];
        }

        double speedEpsilon = Math.min(
                releaseSpeed(fitCoast(forwardSpeeds, forward), POSITION_TOLERANCE),
                releaseSpeed(fitCoast(strafeSpeeds, strafe), POSITION_TOLERANCE)
        );

        double angularEpsilon = releaseSpeed(fitCoast(angularSpeeds, angular), headingTolerance);

        double carried = 0, carriedAngle = 0;

        for (int round = 0; round < MAX_CORRECTIONS && opModeIsActive(); round++) {

            final double gate = speedEpsilon;

            double[] checkForward = retry("checking forward at " + format(gate, 2), () -> checkLeg(gate, 1, 0));
            double[] checkStrafe = retry("checking strafe at " + format(gate, 2), () -> checkLeg(gate, 0, 1));

            carried = Math.max(checkForward[1], checkStrafe[1]);

            if (carried <= POSITION_TOLERANCE) break;

            //coast grows roughly with the square of speed, so this lands just under rather
            // than over the position tolerance
            speedEpsilon = gate * Math.sqrt(POSITION_TOLERANCE / carried);
        }

        for (int round = 0; round < MAX_CORRECTIONS && opModeIsActive(); round++) {

            final double gate = angularEpsilon;

            double[] check = retry("checking turn at " + format(gate, 2), () -> checkAngularLeg(gate));

            carriedAngle = check[1];

            if (carriedAngle <= headingTolerance) break;

            angularEpsilon = gate * Math.sqrt(headingTolerance / carriedAngle);
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== COMPLETION SPEED RESULTS ===");
        telemetry.addData("COMPLETION_SPEED_EPSILON", format(speedEpsilon, 9));
        telemetry.addData("COMPLETION_ANGULAR_SPEED_EPSILON", format(angularEpsilon, 9));

        telemetry.addLine();

        telemetry.addData("carried", format(carried, 3) + " in of " + POSITION_TOLERANCE
                + ", " + format(Math.toDegrees(carriedAngle), 3) + " deg of " + HEADING_TOLERANCE_DEGREES);
        telemetry.addData("sampled up to", format(maxTestSpeed, 2) + " in/s, " + format(maxTestTurn, 2) + " rad/s");
        telemetry.addData("forward speeds", format(forwardSpeeds));
        telemetry.addData("forward coast", format(forward));
        telemetry.addData("strafe speeds", format(strafeSpeeds));
        telemetry.addData("strafe coast", format(strafe));
        telemetry.addData("angular speeds", format(angularSpeeds));
        telemetry.addData("angular coast", format(angular));
        telemetry.update();

        while (opModeIsActive()) ;
    }

    //a fixed range won't work for every robot. Two coasts estimate the stopping
    // distance, after which the real sweep is centered around that estimate.
    private double probeRange(double topSpeed, boolean angular, double tolerance) {

        double[] speeds = new double[2], carried = new double[2];

        for (int i = 0; i < 2; i++) {

            double target = topSpeed * (i == 0 ? PROBE_LOW_SHARE : PROBE_HIGH_SHARE);

            double[] leg = angular
                    ? retry("probing turn at " + format(target, 2), () -> angularCoastLeg(target))
                    : retry("probing at " + format(target, 1), () -> coastLeg(target, 1, 0));

            speeds[i] = leg[0];
            carried[i] = leg[1];
        }

        double rough = releaseSpeed(fitCoast(speeds, carried), tolerance);

        if (!(rough > 0) || Double.isInfinite(rough)) rough = topSpeed * MIN_RANGE_SHARE;

        return MathHelper.clamp(rough * PROBE_RANGE, topSpeed * MIN_RANGE_SHARE, topSpeed * MAX_RANGE_SHARE);
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

            double[] result = leg.run();

            if (!repositioned) return result != null ? result : new double[] {0, 0};
        }

        return new double[] {0, 0};
    }

    private double[] checkLeg(double gateSpeed, double forwardPower, double strafePower) {

        if (reachSpeed(gateSpeed * 2 + 2, forwardPower, strafePower) == null) return null;

        double fell = 0;
        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
            pc.update();

            if (reposition()) return null;

            Pose velocity = pc.getFinalLocalizer().getVelocity();
            fell = Math.hypot(velocity.x, velocity.y);

            if (fell <= gateSpeed) break;
        }

        return new double[] {fell, carry()};
    }

    private double[] checkAngularLeg(double gateSpeed) {

        if (reachAngularSpeed(gateSpeed * 2 + 0.5) == null) return null;

        double fell = 0;
        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
            pc.update();

            if (reposition()) return null;

            fell = Math.abs(pc.getFinalLocalizer().getVelocity().heading);

            if (fell <= gateSpeed) break;
        }

        return new double[] {fell, angularCarry()};
    }

    private double[] coastLeg(double targetSpeed, double forwardPower, double strafePower) {

        Double released = reachSpeed(targetSpeed, forwardPower, strafePower);

        if (released == null) return null;

        return new double[] {released, carry()};
    }

    private double[] angularCoastLeg(double targetSpeed) {

        Double released = reachAngularSpeed(targetSpeed);

        if (released == null) return null;

        return new double[] {released, angularCarry()};
    }

    private double carry() {

        Pose from = pc.getFinalLocalizer().getPose();
        double distance = 0;
        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
            pc.update();

            if (reposition()) break;

            Pose pose = pc.getFinalLocalizer().getPose();
            distance = Math.hypot(pose.x - from.x, pose.y - from.y);

            Pose velocity = pc.getFinalLocalizer().getVelocity();
            if (Math.hypot(velocity.x, velocity.y) < STOPPED_SPEED) break;
        }

        return distance;
    }

    private double angularCarry() {

        double previous = pc.getFinalLocalizer().getPose().heading;
        double turned = 0;
        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
            pc.update();

            if (reposition()) break;

            double heading = pc.getFinalLocalizer().getPose().heading;
            turned += MathHelper.normalizeAngleRad(heading - previous);
            previous = heading;

            if (Math.abs(pc.getFinalLocalizer().getVelocity().heading) < STOPPED_ANGULAR_SPEED) break;
        }

        return Math.abs(turned);
    }

    //eases off as it closes in so it doesn't blow past a low target
    private Double reachSpeed(double targetSpeed, double forwardPower, double strafePower) {

        double start = getRuntime();
        double speed = 0;

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            if (reposition()) return null;

            Pose velocity = pc.getFinalLocalizer().getVelocity();
            speed = Math.hypot(velocity.x, velocity.y);

            if (speed >= targetSpeed) break;

            double share = MathHelper.clamp(1 - speed / targetSpeed, MINIMUM_APPROACH_POWER, 1);

            pc.getChassis().setDrivePower(forwardPower * share, strafePower * share, 0, pc.getFinalLocalizer().getDeltaTime());
        }

        return speed;
    }

    private Double reachAngularSpeed(double targetSpeed) {

        double start = getRuntime();
        double speed = 0;

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            if (reposition()) return null;

            speed = Math.abs(pc.getFinalLocalizer().getVelocity().heading);

            if (speed >= targetSpeed) break;

            double share = MathHelper.clamp(1 - speed / targetSpeed, MINIMUM_APPROACH_POWER, 1);

            pc.getChassis().setDrivePower(0, 0, share, pc.getFinalLocalizer().getDeltaTime());
        }

        return speed;
    }

    private double[] fitCoast(double[] speeds, double[] distances) {

        // Actually Timmy's EXTREMELY tuff knuckles bro.

        double s2 = 0, s3 = 0, s4 = 0, weighted = 0, squareWeighted = 0;

        for (int i = 0; i < speeds.length; i++) {

            double v = speeds[i], d = distances[i];
            double square = v * v;

            s2 += square;
            s3 += square * v;
            s4 += square * square;

            weighted += v * d;
            squareWeighted += square * d;
        }

        double determinant = s2 * s4 - s3 * s3;

        if (determinant == 0) return new double[] {0, 0};

        return new double[] {
                (weighted * s4 - squareWeighted * s3) / determinant,
                (s2 * squareWeighted - s3 * weighted) / determinant
        };
    }

    private double releaseSpeed(double[] coast, double tolerance) {

        double linear = coast[0], square = coast[1];

        if (square <= 0) return linear > 0 ? tolerance / linear : Double.MAX_VALUE;
        if (linear <= 0) return Math.sqrt(tolerance / square);

        return (-linear + Math.sqrt(linear * linear + 4 * square * tolerance)) / (2 * square);
    }

    private boolean repositioned;

    private boolean reposition() {

        if (!gamepad1.b) return false;

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

    private String format(double value, int decimals) {

        double scale = Math.pow(10, decimals);
        double rounded = Math.round(value * scale) / scale;

        return Double.toString(rounded);
    }

    private String format(double[] values) {

        StringBuilder out = new StringBuilder("{");

        for (int i = 0; i < values.length; i++) {

            if (i > 0) out.append(", ");

            out.append(format(values[i], 4));
        }

        return out.append("}").toString();
    }
}
