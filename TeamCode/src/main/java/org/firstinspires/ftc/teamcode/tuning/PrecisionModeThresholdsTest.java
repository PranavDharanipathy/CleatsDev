package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.chassis.BrakingModel;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;

//@Config
@TeleOp(group = "Cleats Tuning")
public class PrecisionModeThresholdsTest extends LinearOpMode {

    private static final int STEPS = 1000;

    private PathController pc;

    private double k1Forward, k2Forward, k1Strafe, k2Strafe, k1Heading, k2Heading;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("Nothing moves. This works the hand off out from the braking model, motion constraints and LQR already in Constants.");
        telemetry.update();

        waitForStart();

        MotionConstraints constraints = pc.getMotionConstraints();
        PoseLQRController lqr = pc.getPoseLQRController();

        k1Forward = -lqr.correctForward(1, 0);
        k2Forward = -lqr.correctForward(0, 1);
        k1Strafe = -lqr.correctStrafe(1, 0);
        k2Strafe = -lqr.correctStrafe(0, 1);
        k1Heading = -lqr.correctHeading(1, 0);
        k2Heading = -lqr.correctHeading(0, 1);

        double entryPositionDistance = 0, entryVelocity = 0;

        for (double angle : new double[] {0, Math.PI / 4d, Math.PI / 2d}) {

            double[] handOff = translationHandOff(angle);

            entryPositionDistance = Math.max(entryPositionDistance, handOff[0]);
            entryVelocity = Math.max(entryVelocity, handOff[1]);
        }

        double[] turn = headingHandOff();

        double entryHeadingError = turn[0];
        double entryAngularVelocity = turn[1];

        double strongest = Math.max(constraints.getAmaxF(), constraints.getAmaxS());

        double exitPositionDistance = Math.max(entryPositionDistance, strongest / Math.min(k1Forward, k1Strafe));
        double exitVelocity = Math.max(entryVelocity, strongest / Math.min(k2Forward, k2Strafe));

        double exitHeadingError = Math.max(entryHeadingError, constraints.getAmaxH() / k1Heading);
        double exitAngularVelocity = Math.max(entryAngularVelocity, constraints.getAmaxH() / k2Heading);

        telemetry.addLine("=== PrecisionModeThresholds ===");
        telemetry.addData("entryPositionDistance", entryPositionDistance);
        telemetry.addData("exitPositionDistance", exitPositionDistance);
        telemetry.addData("entryVelocity", entryVelocity);
        telemetry.addData("exitVelocity", exitVelocity);
        telemetry.addData("entryHeadingError", entryHeadingError);
        telemetry.addData("exitHeadingError", exitHeadingError);
        telemetry.addData("entryAngularVelocity", entryAngularVelocity);
        telemetry.addData("exitAngularVelocity", exitAngularVelocity);

        if (!(k1Forward > 0) || !(k1Strafe > 0) || !(k1Heading > 0)) {
            telemetry.addLine("The LQR in Constants has no gains yet, run TranslationLQRTest and HeadingLQRTest first.");
        }

        telemetry.update();

        while (opModeIsActive()) ;
    }

    //fastest point on the braking curve the LQR can take over from within full power
    private double[] translationHandOff(double angle) {

        BrakingModel model = pc.getBrakingModel();
        MotionConstraints constraints = pc.getMotionConstraints();

        double cos = Math.abs(Math.cos(angle)), sin = Math.abs(Math.sin(angle));
        double top = pc.getMecanumProfile().getMaxVelocity(angle);

        double distance = 0, speed = 0;

        for (int i = 1; i <= STEPS; i++) {

            double v = top * i / STEPS;
            double e = model.getStoppingDistance(angle, v);

            if (Math.abs(k1Forward * e - k2Forward * v) * cos >= constraints.getAmaxF()) break;
            if (Math.abs(k1Strafe * e - k2Strafe * v) * sin >= constraints.getAmaxS()) break;

            distance = e;
            speed = v;
        }

        return new double[] {distance, speed};
    }

    private double[] headingHandOff() {

        BrakingModel model = pc.getBrakingModel();
        MotionConstraints constraints = pc.getMotionConstraints();

        double top = constraints.getVmaxH();

        double angle = 0, speed = 0;

        for (int i = 1; i <= STEPS; i++) {

            double w = top * i / STEPS;
            double e = model.getAngularStoppingDistance(w);

            if (Math.abs(k1Heading * e - k2Heading * w) >= constraints.getAmaxH()) break;

            angle = e;
            speed = w;
        }

        return new double[] {angle, speed};
    }
}
