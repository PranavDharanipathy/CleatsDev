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
public class LQROvershootDiagnosticTest extends LinearOpMode {

    private static final double SAMPLE_DURATION = 2;

    //a standing robot's ticks never change, so the wobble only shows up against motion
    private static final double CREEP_POWER = 0.25;
    private static final double SPIN_UP_SECONDS = 0.5;

    //how many standard deviations of noise count as already settled down,
    //this is an educated guess, not derived from data
    private static final double ALREADY_CLOSE_NOISE_MULTIPLIER = 30; //determined using three sigma rule

    private Telemetry telemetry;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Clear about 3 ft diagonally forward and to the right.");
        telemetry.addLine("Press start, the robot will creep in a straight line and measure how far the localizer strays from it.");
        telemetry.update();

        waitForStart();

        Wobble x = new Wobble(), y = new Wobble(), heading = new Wobble();

        //heading is measured against the first sample, otherwise sitting near +/-180 wraps causing problems
        Double referenceHeading = null;

        double startTime = getRuntime();

        while (opModeIsActive() && getRuntime() - startTime < SPIN_UP_SECONDS + SAMPLE_DURATION) {

            pc.update();
            pc.getChassis().setDrivePowerBypassRamp(CREEP_POWER, CREEP_POWER, 0);

            double t = getRuntime() - startTime;

            //the first moments are acceleration, not steady motion
            if (t < SPIN_UP_SECONDS) continue;

            Pose pose = pc.getFinalLocalizer().getPose();

            if (referenceHeading == null) referenceHeading = pose.heading;

            x.add(t, pose.x);
            y.add(t, pose.y);
            heading.add(t, MathHelper.normalizeAngleRad(pose.heading - referenceHeading));

            telemetry.addData("sampling", "%.3f / %.3f sec", t - SPIN_UP_SECONDS, SAMPLE_DURATION);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        final double positionNoise = Math.sqrt(x.variance() + y.variance());
        final double headingNoise = Math.sqrt(heading.variance());

        double travelled = Math.hypot(x.speed(), y.speed()) * SAMPLE_DURATION;

        if (travelled <= positionNoise) {

            telemetry.addLine("The robot moved less than the stray being measured, so there is nothing here to measure.");
            telemetry.update();

            while (opModeIsActive()) ;
            return;
        }

        final double positionAlreadyCloseThreshold = positionNoise * ALREADY_CLOSE_NOISE_MULTIPLIER;
        final double headingAlreadyCloseThreshold = headingNoise * ALREADY_CLOSE_NOISE_MULTIPLIER;

        telemetry.addLine("=== FOR TranslationLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD_POSITION (in)", positionAlreadyCloseThreshold);

        telemetry.addLine("=== FOR HeadingLQRTest ===");
        telemetry.addData("ALREADY_CLOSE_THRESHOLD_HEADING (rad)", headingAlreadyCloseThreshold);

        telemetry.update();

        while (opModeIsActive()) ;
    }

    private static class Wobble {

        private double n, meanT, meanValue, m2T, m2Value, covariance;

        void add(double t, double value) {

            n++;

            //welford's algorithm for the win!
            double dt = t - meanT;
            meanT += dt / n;
            m2T += dt * (t - meanT);

            double dValue = value - meanValue;
            meanValue += dValue / n;
            m2Value += dValue * (value - meanValue);

            covariance += dt * (value - meanValue);
        }

        double speed() {
            return m2T > 0 ? covariance / m2T : 0;
        }

        //what is left once the steady motion is taken back out
        double variance() {

            if (n < 3 || m2T <= 0) return 0;

            double residual = m2Value - speed() * covariance;

            return Math.max(0, residual) / (n - 2);
        }
    }
}
