package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

/// Checks that the Pinpoint and the chassis agree on every direction, (the speed tuners
/// can't tell if it's correct or not)
public class DirectionCheck {

    private static final double NUDGE_DISTANCE = 3; //inches
    private static final double NUDGE_ANGLE = Math.toRadians(15);

    private final LinearOpMode opMode;
    private final PathController pc;

    public DirectionCheck(LinearOpMode opMode, PathController pc) {

        this.opMode = opMode;
        this.pc = pc;
    }

    /// @return what's wrong (null if nothing is)
    public String run() {

        MecanumProfile profile = pc.getMecanumProfile();

        for (int sign = 1; sign >= -1; sign -= 2) {

            double[] moved = nudge(sign, 0, 0, NUDGE_DISTANCE, profile.getMaxAcceleration(0));

            String way = sign > 0 ? "forward" : "backward";

            if (Math.abs(moved[0]) < NUDGE_DISTANCE / 2d && Math.abs(moved[1]) >= NUDGE_DISTANCE) return "Drove " + way + " but the Pinpoint saw it move sideways. If the robot really went " + way + ", the forward and strafe pods are swapped on the Pinpoint. If not, fix the motor directions.";

            String problem = translationProblem(sign * moved[0], way, sign > 0 ? "backward" : "forward", "forward");
            if (problem != null) return problem;
        }

        for (int sign = 1; sign >= -1; sign -= 2) {

            double[] moved = nudge(0, sign, 0, NUDGE_DISTANCE, profile.getMaxAcceleration(Math.PI / 2d));

            String problem = translationProblem(-sign * moved[1], sign > 0 ? "right" : "left", sign > 0 ? "left" : "right", "strafe");
            if (problem != null) return problem;
        }

        for (int sign = 1; sign >= -1; sign -= 2) {

            double[] moved = nudge(0, 0, sign, NUDGE_ANGLE, pc.getMotionConstraints().getAmaxH());

            String way = sign > 0 ? "counterclockwise" : "clockwise";
            String opposite = sign > 0 ? "clockwise" : "counterclockwise";

            if (Math.abs(moved[2]) < NUDGE_ANGLE / 2d) return "Turned " + way + " but the Pinpoint heading barely moved. If the robot did turn, the Pinpoint isn't reading its heading. If not, fix the motor directions.";

            if (sign * moved[2] < 0) return "Turned " + way + " but the Pinpoint saw " + opposite + ". If the robot really turned " + way + " (seen from above), the Pinpoint is mounted upside down. If not, fix the motor directions.";
        }

        return null;
    }

    private String translationProblem(double along, String way, String opposite, String pod) {

        if (Math.abs(along) < NUDGE_DISTANCE / 2d) return "Drove " + way + " but the Pinpoint barely moved. If the robot did move, check the " + pod + " pod's cable and port. If not, fix the motor directions.";

        if (along < 0) return "Drove " + way + " but the Pinpoint saw it go " + opposite + ". If the robot really went " + way + ", flip the " + pod + " pod direction in Constants. If not, fix the motor directions.";

        return null;
    }

    private double[] nudge(double forward, double strafe, double turn, double size, double acceleration) {

        pc.update();
        Pose start = pc.getPose().copy();

        //the ramp's time to full power plus the nudge at full acceleration
        double maxTime = 1d / pc.getMotionConstraints().getAntiSlipRampRate() + Math.sqrt(2d * size / acceleration);

        double pushTime = opMode.getRuntime();

        while (opMode.opModeIsActive() && opMode.getRuntime() - pushTime < maxTime) {

            pc.update();

            if (reached(start, turn != 0) >= size) break;

            pc.getChassis().setDrivePower(forward, strafe, turn, pc.getFinalLocalizer().getDeltaTime());

            opMode.telemetry.addLine("Checking drive directions");
            opMode.telemetry.update();
        }

        double farthest = reached(start, turn != 0);
        double brakeTime = opMode.getRuntime();

        while (opMode.opModeIsActive() && opMode.getRuntime() - brakeTime < maxTime) {

            pc.update();

            double now = reached(start, turn != 0);
            if (now <= farthest) break;
            farthest = now;

            pc.getChassis().setDrivePower(-forward, -strafe, -turn, pc.getFinalLocalizer().getDeltaTime());
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        return movedSince(start);
    }

    private double reached(Pose start, boolean turning) {

        double[] moved = movedSince(start);
        return turning ? Math.abs(moved[2]) : Math.hypot(moved[0], moved[1]);
    }

    private double[] movedSince(Pose start) {

        Pose now = pc.getPose();

        double dx = now.x - start.x;
        double dy = now.y - start.y;

        return new double[] {
                dx * Math.cos(start.heading) + dy * Math.sin(start.heading),
                -dx * Math.sin(start.heading) + dy * Math.cos(start.heading),
                MathHelper.normalizeAngleRad(now.heading - start.heading)
        };
    }
}
