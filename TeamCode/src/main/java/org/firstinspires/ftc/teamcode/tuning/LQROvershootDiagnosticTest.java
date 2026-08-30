package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class LQROvershootDiagnosticTest extends LinearOpMode {

    private static final double SAMPLE_DURATION = 2;

    //how many standard deviations of stationary noise count as already settled down,
    //this is an educated guess, not derived from data
    private static final double ALREADY_CLOSE_NOISE_MULTIPLIER = 30; //determined using three sigma rule

    //minimums
    private static final double MIN_POSITION_ALREADY_CLOSE_THRESHOLD = 1;
    private static final double MIN_HEADING_ALREADY_CLOSE_THRESHOLD = Math.toRadians(3);

    private Telemetry telemetry;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController();

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Place the robot down and do not touch it.");
        telemetry.addLine("Press start, the robot will not move.");
        telemetry.update();

        waitForStart();

        double avgX = 0, avgY = 0, avgHeading = 0;
        double m2X = 0, m2Y = 0, m2Heading = 0;

        int count = 0;

        double startTime = getRuntime();

        while (opModeIsActive() && getRuntime() - startTime < SAMPLE_DURATION) {

            pc.update();
            pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

            Pose pose = pc.getFinalLocalizer().getPose();

            count++;

            //welford's algorithm for the win!
            double dx = pose.x - avgX;
            avgX += dx / count;
            m2X += dx * (pose.x - avgX);

            double dy = pose.y - avgY;
            avgY += dy / count;
            m2Y += dy * (pose.y - avgY);

            double dh = pose.heading - avgHeading;
            avgHeading += dh / count;
            m2Heading += dh * (pose.heading - avgHeading);

            telemetry.addData("sampling", "%.3f / %.3f sec", getRuntime() - startTime, SAMPLE_DURATION);
            telemetry.update();
        }

        double xVariance = count > 1 ? m2X / (count - 1) : 0;
        double yVariance = count > 1 ? m2Y / (count - 1) : 0;
        double headingVariance = count > 1 ? m2Heading / (count - 1) : 0;

        final double positionNoise = Math.sqrt(xVariance + yVariance);
        final double headingNoise = Math.sqrt(headingVariance);

        final double positionAlreadyCloseThreshold = Math.max(MIN_POSITION_ALREADY_CLOSE_THRESHOLD, positionNoise * ALREADY_CLOSE_NOISE_MULTIPLIER);
        final double headingAlreadyCloseThreshold = Math.max(MIN_HEADING_ALREADY_CLOSE_THRESHOLD, headingNoise * ALREADY_CLOSE_NOISE_MULTIPLIER);

        telemetry.addLine("=== FOR TranslationLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD (in)", positionAlreadyCloseThreshold);

        telemetry.addLine("=== FOR HeadingLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD (rad)", headingAlreadyCloseThreshold);

        telemetry.update();

        while (opModeIsActive()) ;
    }
}
