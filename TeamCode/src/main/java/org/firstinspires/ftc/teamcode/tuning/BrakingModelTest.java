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
public class BrakingModelTest extends LinearOpMode {

    public static int SAMPLES = 5;

    public static double MAX_TEST_SPEED = 50; //inches per second
    public static double MAX_TEST_ANGULAR_SPEED = 8; //radians per second

    private static final double STOPPED_SPEED = 1;
    private static final double STOPPED_ANGULAR_SPEED = 0.15;
    private static final double MAX_LEG_TIME = 6;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Give the robot a long clear path in every direction.");
        telemetry.addLine("It will accelerate and brake hard at various speeds.");
        telemetry.addLine("Press A between legs to continue.");
        telemetry.update();

        waitForStart();

        double[] speeds = new double[SAMPLES];
        double[] forward = new double[SAMPLES];
        double[] strafe = new double[SAMPLES];
        double[] diagonal = new double[SAMPLES];

        double[] angularSpeeds = new double[SAMPLES];
        double[] angular = new double[SAMPLES];

        for (int i = 0; i < SAMPLES && opModeIsActive(); i++) {

            speeds[i] = MAX_TEST_SPEED * (i + 1) / SAMPLES;
            angularSpeeds[i] = MAX_TEST_ANGULAR_SPEED * (i + 1) / SAMPLES;

            forward[i] = translationLeg(speeds[i], 1, 0, telemetry, "forward");
            strafe[i] = translationLeg(speeds[i], 0, 1, telemetry, "strafe");
            diagonal[i] = translationLeg(speeds[i], Math.sqrt(0.5), Math.sqrt(0.5), telemetry, "diagonal");
            angular[i] = angularLeg(angularSpeeds[i], telemetry);
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== BrakingModel ===");
        telemetry.addData("speeds", format(speeds));
        telemetry.addData("forwardDistances", format(forward));
        telemetry.addData("strafeDistances", format(strafe));
        telemetry.addData("diagonalDistances", format(diagonal));
        telemetry.addData("angularSpeeds", format(angularSpeeds));
        telemetry.addData("angularDistances", format(angular));
        telemetry.addLine("latency: one control loop of pose staleness, usually 0.01 to 0.03");
        telemetry.addLine("margin: how close counts as arrived, try a third of your position tolerance");
        telemetry.update();

        while (opModeIsActive()) ;
    }

    private double translationLeg(double targetSpeed, double forwardPower, double strafePower, Telemetry telemetry, String name) {

        waitForA(telemetry, name + " leg, target " + targetSpeed + " in/s");

        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            Pose velocity = pc.getFinalLocalizer().getVelocity();
            if (Math.hypot(velocity.x, velocity.y) >= targetSpeed) break;

            pc.getChassis().setDrivePower(forwardPower, strafePower, 0, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData(name + " accelerating", Math.hypot(velocity.x, velocity.y));
            telemetry.update();
        }

        Pose brakeStart = pc.getFinalLocalizer().getPose();
        double distance = 0;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            pc.update();

            Pose pose = pc.getFinalLocalizer().getPose();
            Pose velocity = pc.getFinalLocalizer().getVelocity();

            distance = Math.hypot(pose.x - brakeStart.x, pose.y - brakeStart.y);

            if (Math.hypot(velocity.x, velocity.y) < STOPPED_SPEED) break;

            pc.getChassis().setDrivePower(-forwardPower, -strafePower, 0, pc.getFinalLocalizer().getDeltaTime());

            telemetry.addData(name + " braking from " + targetSpeed, distance);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return distance;
    }

    private double angularLeg(double targetSpeed, Telemetry telemetry) {

        waitForA(telemetry, "turn leg, target " + targetSpeed + " rad/s");

        double start = getRuntime();

        while (opModeIsActive() && getRuntime() - start < MAX_LEG_TIME) {

            pc.update();

            if (Math.abs(pc.getFinalLocalizer().getVelocity().heading) >= targetSpeed) break;

            pc.getChassis().setDrivePower(0, 0, 1, pc.getFinalLocalizer().getDeltaTime());
            telemetry.addData("turn accelerating", pc.getFinalLocalizer().getVelocity().heading);
            telemetry.update();
        }

        double brakeStart = pc.getFinalLocalizer().getPose().heading;
        double turned = 0, previous = brakeStart;
        double brakeTime = getRuntime();

        while (opModeIsActive() && getRuntime() - brakeTime < MAX_LEG_TIME) {

            pc.update();

            double heading = pc.getFinalLocalizer().getPose().heading;
            turned += MathHelper.normalizeAngleRad(heading - previous);
            previous = heading;

            if (Math.abs(pc.getFinalLocalizer().getVelocity().heading) < STOPPED_ANGULAR_SPEED) break;

            pc.getChassis().setDrivePower(0, 0, -1, pc.getFinalLocalizer().getDeltaTime());
            telemetry.addData("turn braking from " + targetSpeed, Math.abs(turned));
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);
        return Math.abs(turned);
    }

    private void waitForA(Telemetry telemetry, String label) {

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        while (opModeIsActive() && !gamepad1.a) {
            pc.update();
            telemetry.addLine("Press A to run: " + label);
            telemetry.update();
        }

        while (opModeIsActive() && gamepad1.a) pc.update();
    }

    private String format(double[] values) {

        StringBuilder out = new StringBuilder("{");

        for (int i = 0; i < values.length; i++) {

            if (i > 0) out.append(", ");
            out.append(String.format("%.4f", values[i]));
        }

        return out.append("}").toString();
    }
}
