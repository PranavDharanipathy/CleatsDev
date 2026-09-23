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
import org.firstinspires.ftc.teamcode.util.PoseLQRTuner;

//@Config
@TeleOp(group = "Cleats Tuning")
public class LQROvershootDiagnosticTest extends LinearOpMode {

    //a thousandth of full power a loop, so it's caught just as it starts moving
    private static final double RAMP_PER_LOOP = 0.001;

    private Telemetry telemetry;

    private PathController pc;
    private PoseLQRTuner translationTuner, headingTuner;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Clear about a foot all around the robot.");
        telemetry.addLine("Press start, the robot will push a little harder each moment until it just starts to move, both ways on every axis.");
        telemetry.update();

        waitForStart();

        //the same loop rate limit the LQR tests end up at
        translationTuner = new PoseLQRTuner(pc.getMotionConstraints(), TranslationLQRTest.LOOP_ITERATIONS_PER_TIME_CONSTANT);
        headingTuner = new PoseLQRTuner(pc.getMotionConstraints(), HeadingLQRTest.LOOP_ITERATIONS_PER_TIME_CONSTANT);

        double forward = Math.max(breakaway(1, 0, 0), breakaway(-1, 0, 0));
        double strafe = Math.max(breakaway(0, 1, 0), breakaway(0, -1, 0));
        double turn = Math.max(breakaway(0, 0, 1), breakaway(0, 0, -1));

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        double k1Forward = Math.sqrt(translationTuner.getLastQPositionForward());
        double k1Strafe = Math.sqrt(translationTuner.getLastQPositionStrafe());
        double k1Heading = Math.sqrt(headingTuner.getLastQPositionHeading());

        //below breakaway the LQR can't move the robot, so it can't promise better
        double forwardStuck = forward * pc.getMecanumProfile().getMaxAcceleration(0) / k1Forward;
        double strafeStuck = strafe * pc.getMecanumProfile().getMaxAcceleration(Math.PI / 2d) / k1Strafe;

        final double positionAlreadyCloseThreshold = Math.hypot(forwardStuck, strafeStuck);
        final double headingAlreadyCloseThreshold = turn * pc.getMotionConstraints().getAmaxH() / k1Heading;

        telemetry.addLine("=== FOR TranslationLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD_POSITION (in)", positionAlreadyCloseThreshold);

        telemetry.addLine("=== FOR HeadingLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD_HEADING (rad)", headingAlreadyCloseThreshold);

        telemetry.update();

        while (opModeIsActive()) ;
    }

    private double breakaway(double forward, double strafe, double turn) {

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        //a time constant to stop, then one to see how much a still reading moves
        double settle = pc.getMotionConstraints().getVmaxF() / pc.getMotionConstraints().getAmaxF();

        double begin = getRuntime();
        while (opModeIsActive() && getRuntime() - begin < settle) update();

        Pose last = pc.getPose().copy();
        double still = 0;

        begin = getRuntime();

        while (opModeIsActive() && getRuntime() - begin < settle) {

            update();

            still = Math.max(still, moved(last, turn != 0));
            last = pc.getPose().copy();
        }

        double power = 0;

        while (opModeIsActive() && power < 1) {

            update();

            boolean going = moved(last, turn != 0) > 2 * still;
            last = pc.getPose().copy();

            if (going) break;

            power += RAMP_PER_LOOP;

            pc.getChassis().setDrivePowerBypassRamp(forward * power, strafe * power, turn * power);

            telemetry.addData("pushing", "%.3f", power);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        return power;
    }

    private double moved(Pose from, boolean turning) {

        Pose pose = pc.getPose();

        if (turning) return Math.abs(MathHelper.normalizeAngleRad(pose.heading - from.heading));

        return Math.hypot(pose.x - from.x, pose.y - from.y);
    }

    private void update() {

        pc.update();

        double dt = pc.getFinalLocalizer().getDeltaTime();

        translationTuner.update(0, 0, 0, 0, 0, 0, dt);
        headingTuner.update(0, 0, 0, 0, 0, 0, dt);
    }
}
