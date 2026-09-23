package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.path.Turn;
import org.firstinspires.ftc.teamcode.util.MathHelper;

//@Config
@TeleOp(group = "Cleats Tuning")
public class RotationHoldAuthorityTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        PathController pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("Nothing moves. This works the hold's share out from the braking model and motion constraints already in Constants.");
        telemetry.update();

        waitForStart();

        double stop = pc.getBrakingModel().getAngularStoppingDistance(pc.getMotionConstraints().getVmaxH());
        double slack = Math.min(pc.getBrakingModel().getAngularMargin(), Turn.degrees(90).getHeadingTolerance());

        telemetry.addLine("=== ROTATION HOLD AUTHORITY ===");

        if (!(stop > 0)) {

            telemetry.addLine("There is no turn braking in Constants yet, run BrakingModelTest first.");
            telemetry.update();

            while (opModeIsActive()) ;
            return;
        }

        //a hold capped at h stretches that stop by up to h * sqrt(2)
        telemetry.addData("ROTATION_HOLD_AUTHORITY", MathHelper.clamp(slack / (Math.sqrt(2) * stop), 0, 1));
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
