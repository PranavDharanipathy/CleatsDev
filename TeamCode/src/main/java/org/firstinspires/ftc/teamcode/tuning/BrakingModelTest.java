package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.chassis.BrakingModel;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class BrakingModelTest extends LinearOpMode {

    public static int SAMPLES = 5;

    public static double HEADROOM = 1.5;

    private static final double MAX_LEG_TIME = 6;

    private PathController pc;
    private Telemetry telemetry;

    private double loops, loopSum, loopSquares;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Give the robot a long clear path in every direction.");
        telemetry.addLine("It will accelerate and brake hard at various speeds.");
        telemetry.addLine("Drive with the sticks while waiting, A to run each leg.");
        telemetry.update();

        waitForStart();

        //the three tables share the same speed axis so we have to pick the smallest one
        double topSpeed = Math.min(pc.getMotionConstraints().getVmaxF(), Math.min(pc.getMotionConstraints().getVmaxS(), pc.getMotionConstraints().getVmaxD()));

        double topAngularSpeed = pc.getMotionConstraints().getVmaxH();

        double[] speeds = new double[SAMPLES];
        double[] forward = new double[SAMPLES];
        double[] strafe = new double[SAMPLES];
        double[] diagonal = new double[SAMPLES];

        double[] angularSpeeds = new double[SAMPLES];
        double[] angular = new double[SAMPLES];

        for (int i = 0; i < SAMPLES && opModeIsActive(); i++) {

            //top speed is only ever approached, so the fastest sample stops a step short
            speeds[i] = topSpeed * (i + 1) / (SAMPLES + 1);
            angularSpeeds[i] = topAngularSpeed * (i + 1) / (SAMPLES + 1);

            forward[i] = translationLeg(speeds[i], 1, 0, "forward");
            strafe[i] = translationLeg(speeds[i], 0, 1, "strafe");
            diagonal[i] = translationLeg(speeds[i], Math.sqrt(0.5), Math.sqrt(0.5), "diagonal");

            angular[i] = angularLeg(angularSpeeds[i]);
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        double margin = 0, angularMargin = 0;

        String missed = firstMissing(forward, "forward", strafe, "strafe", diagonal, "diagonal", angular, "turn");

        if (missed == null) {

            BrakingModel model = new BrakingModel(speeds, forward, strafe, diagonal, angularSpeeds, angular, 0, 0);

            //the slowest loop the control hub normally takes, three sigma out
            double mean = loopSum / loops;
            double loop = mean + 3 * Math.sqrt(Math.max(0, loopSquares / loops - mean * mean));

            //bang-bang can't move less than one loop of push plus its brake, tighter hunts
            for (double angle : new double[] {0, Math.PI / 4d, Math.PI / 2d}) {

                double push = pc.getMecanumProfile().getMaxAcceleration(angle) * loop;
                margin = Math.max(margin, push * loop / 2d + model.getStoppingDistance(angle, push));
            }

            double spin = pc.getMotionConstraints().getAmaxH() * loop;
            angularMargin = spin * loop / 2d + model.getAngularStoppingDistance(spin);

            margin *= HEADROOM;
            angularMargin *= HEADROOM;
        }

        telemetry.addLine("=== BRAKING MODEL RESULTS ===");

        //a zero in a table means a leg never got up to speed
        if (missed != null) telemetry.addLine("The " + missed + " legs did not all reach their speed, so there is no margin. Give that direction more room.");

        addTable("speeds", speeds);
        addTable("forwardDistances", forward);
        addTable("strafeDistances", strafe);
        addTable("diagonalDistances", diagonal);
        addTable("angularSpeeds", angularSpeeds);
        addTable("angularDistances", angular);

        telemetry.addData("margin", "%.9f", margin);
        telemetry.addData("angularMargin", "Math.toRadians(%.9f)", Math.toDegrees(angularMargin));
        telemetry.update();

        while (opModeIsActive()) ;
    }

    private String firstMissing(Object... tablesAndNames) {

        for (int i = 0; i < tablesAndNames.length; i += 2) {

            for (double value : (double[]) tablesAndNames[i]) {
                if (value <= 0) return (String) tablesAndNames[i + 1];
            }
        }

        return null;
    }

    private double translationLeg(double targetSpeed, double forwardPower, double strafePower, String name) {

        waitForA(name + " leg");

        double start = getRuntime();

        double alongX = 0, alongY = 0;

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            update();

            Pose velocity = pc.getVelocity();
            double speed = Math.hypot(velocity.x, velocity.y);

            if (speed >= targetSpeed) {

                alongX = velocity.x / speed;
                alongY = velocity.y / speed;

                break;
            }

            pc.getChassis().setDrivePower(forwardPower, strafePower, 0, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData(name + " accelerating", "%.2f / %.2f in/s", speed, targetSpeed);
            telemetry.update();
        }

        if (alongX == 0 && alongY == 0) return 0;

        Pose brakeStart = pc.getPose();

        double distance = 0;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            update();

            Pose pose = pc.getPose();

            double travelled = (pose.x - brakeStart.x) * alongX + (pose.y - brakeStart.y) * alongY;

            if (travelled < distance) break;

            distance = travelled;

            pc.getChassis().setDrivePower(-forwardPower, -strafePower, 0, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData(name + " braking", "%.2f in from %.2f in/s", distance, targetSpeed);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return distance;
    }

    private double angularLeg(double targetSpeed) {

        waitForA("turn leg");

        double start = getRuntime();

        double previous = pc.getHeading();
        double spin = 0;

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            update();

            previous = pc.getHeading();

            double angularVelocity = pc.getVelocity().heading;

            if (Math.abs(angularVelocity) >= targetSpeed) {

                spin = Math.signum(angularVelocity);
                break;
            }

            pc.getChassis().setDrivePower(0, 0, 1, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData("turn accelerating", "%.2f / %.2f rad/s", angularVelocity, targetSpeed);
            telemetry.update();
        }

        if (spin == 0) return 0;

        double rotated = 0;
        double turned = 0;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            update();

            double heading = pc.getHeading();
            rotated += MathHelper.normalizeAngleRad(heading - previous) * spin;
            previous = heading;

            if (rotated < turned) break;

            turned = rotated;

            pc.getChassis().setDrivePower(0, 0, -1, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData("turn braking", "%.4f rad from %.2f rad/s", turned, targetSpeed);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return turned;
    }

    private void update() {

        pc.update();

        double dt = pc.getFinalLocalizer().getDeltaTime();

        if (dt <= 0) return;

        loops++;
        loopSum += dt;
        loopSquares += dt * dt;
    }

    private void waitForA(String label) {

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        //we read pose even in init
        update();

        while (opModeIsActive() && !gamepad1.a) {

            update();

            pc.getChassis().driveFromJoystick(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("A to run", label);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        while (opModeIsActive() && gamepad1.a) update();
    }

    private void addTable(String caption, double[] values) {

        StringBuilder pattern = new StringBuilder("{");
        Object[] boxed = new Object[values.length];

        for (int i = 0; i < values.length; i++) {

            if (i > 0) pattern.append(", ");

            pattern.append("%.4f");
            boxed[i] = values[i];
        }

        telemetry.addData(caption, pattern.append("}").toString(), boxed);
    }
}
