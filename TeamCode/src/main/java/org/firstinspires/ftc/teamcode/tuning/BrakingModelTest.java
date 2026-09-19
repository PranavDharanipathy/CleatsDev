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

    public static double HOLD_SECONDS = 2;

    public static double HEADROOM = 1.5;

    private static final double STOPPED_SPEED = 1;
    private static final double STOPPED_ANGULAR_SPEED = 0.15;
    private static final double MAX_LEG_TIME = 6;

    private static final double MARGIN_DISTANCE = 24;
    private static final double MARGIN_DEGREES = 90;

    private PathController pc;
    private Telemetry telemetry;

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

            speeds[i] = topSpeed * (i + 1) / SAMPLES;
            angularSpeeds[i] = topAngularSpeed * (i + 1) / SAMPLES;

            forward[i] = translationLeg(speeds[i], 1, 0, "forward");
            strafe[i] = translationLeg(speeds[i], 0, 1, "strafe");
            diagonal[i] = translationLeg(speeds[i], Math.sqrt(0.5), Math.sqrt(0.5), "diagonal");
            angular[i] = angularLeg(angularSpeeds[i]);
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        double margin = 0, angularMargin = 0;

        if (usable(forward) && usable(strafe) && usable(diagonal) && usable(angular)) {

            BrakingModel model = new BrakingModel(speeds, forward, strafe, diagonal, angularSpeeds, angular, 0, 0);

            margin = marginLeg(model) * HEADROOM;
            angularMargin = angularMarginLeg(model) * HEADROOM;
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== BRAKING MODEL RESULTS ===");
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

    private boolean usable(double[] values) {

        for (double value : values) if (value <= 0) return false;

        return true;
    }

    private double translationLeg(double targetSpeed, double forwardPower, double strafePower, String name) {

        waitForA(name + " leg");

        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            Pose velocity = pc.getVelocity();
            if (Math.hypot(velocity.x, velocity.y) >= targetSpeed) break;

            pc.getChassis().setDrivePower(forwardPower, strafePower, 0, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData(name + " accelerating", "%.2f / %.2f in/s", Math.hypot(velocity.x, velocity.y), targetSpeed);
            telemetry.update();
        }

        Pose brakeStart = pc.getPose();
        double distance = 0;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            pc.update();

            Pose pose = pc.getPose();
            Pose velocity = pc.getVelocity();

            distance = Math.hypot(pose.x - brakeStart.x, pose.y - brakeStart.y);

            if (Math.hypot(velocity.x, velocity.y) < STOPPED_SPEED) break;

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

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            if (Math.abs(pc.getVelocity().heading) >= targetSpeed) break;

            pc.getChassis().setDrivePower(0, 0, 1, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData("turn accelerating", "%.2f / %.2f rad/s", pc.getVelocity().heading, targetSpeed);
            telemetry.update();
        }

        double previous = pc.getHeading();
        double turned = 0;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            pc.update();

            double heading = pc.getHeading();
            turned += MathHelper.normalizeAngleRad(heading - previous);
            previous = heading;

            if (Math.abs(pc.getVelocity().heading) < STOPPED_ANGULAR_SPEED) break;

            pc.getChassis().setDrivePower(0, 0, -1, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData("turn braking", "%.4f rad from %.2f rad/s", Math.abs(turned), targetSpeed);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return Math.abs(turned);
    }

    //margin is the bangbang's deadband
    private double marginLeg(BrakingModel model) {

        waitForA("margin leg");

        Pose start = pc.getPose();

        double cos = Math.cos(start.heading), sin = Math.sin(start.heading);

        double worst = 0;
        double until = 0;
        boolean crossed = false;

        double begin = getRuntime();

        while (opModeIsActive() && getRuntime() - begin < MAX_LEG_TIME + HOLD_SECONDS) {

            pc.update();

            double travelled = (pc.getX() - start.x) * cos + (pc.getY() - start.y) * sin;
            double error = MARGIN_DISTANCE - travelled;

            Pose velocity = pc.getVelocity();
            double closing = velocity.x * cos + velocity.y * sin;

            double stopping = model.getStoppingDistance(0, Math.abs(closing));

            boolean toward = error * closing > 0;
            double command = toward && Math.abs(error) <= stopping ? -Math.signum(closing) : Math.signum(error);

            pc.getChassis().setDrivePower(command, 0, 0, pc.getFinalLocalizer().getDeltaTime());

            if (!crossed && error <= 0) {

                crossed = true;
                until = getRuntime() + HOLD_SECONDS;
            }

            if (crossed) {

                worst = Math.max(worst, Math.abs(error));

                telemetry.addData("margin wobble", "%.4f in", worst);
                telemetry.update();

                if (getRuntime() >= until) break;
            }
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return worst;
    }

    private double angularMarginLeg(BrakingModel model) {

        waitForA("angular margin leg");

        double target = pc.getHeading() + Math.toRadians(MARGIN_DEGREES);

        double worst = 0;
        double until = 0;
        boolean crossed = false;

        double begin = getRuntime();

        while (opModeIsActive() && getRuntime() - begin < MAX_LEG_TIME + HOLD_SECONDS) {

            pc.update();

            double error = MathHelper.normalizeAngleRad(target - pc.getHeading());
            double velocity = pc.getVelocity().heading;

            double stopping = model.getAngularStoppingDistance(Math.abs(velocity));

            boolean toward = error * velocity > 0;
            double command = toward && Math.abs(error) <= stopping ? -Math.signum(velocity) : Math.signum(error);

            pc.getChassis().setDrivePower(0, 0, command, pc.getFinalLocalizer().getDeltaTime());

            if (!crossed && error <= 0) {

                crossed = true;
                until = getRuntime() + HOLD_SECONDS;
            }

            if (crossed) {

                worst = Math.max(worst, Math.abs(error));

                telemetry.addData("angular margin wobble", "%.4f deg", Math.toDegrees(worst));
                telemetry.update();

                if (getRuntime() >= until) break;
            }
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return worst;
    }

    private void waitForA(String label) {

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        //a leg reads the pose before its first update, so it must not be the one from init
        pc.update();

        while (opModeIsActive() && !gamepad1.a) {

            pc.update();

            pc.getChassis().driveFromJoystick(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("A to run", label);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        while (opModeIsActive() && gamepad1.a) pc.update();
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
