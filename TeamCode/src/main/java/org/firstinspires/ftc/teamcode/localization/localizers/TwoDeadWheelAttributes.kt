package org.firstinspires.ftc.teamcode.localization.localizers

import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.localization.Encoder
import org.firstinspires.ftc.teamcode.localization.Odometer

data class TwoDeadWheelAttributes(

    val forwardEncoderName: String,
    val strafeEncoderName: String,
    val imuName: String,

    val forwardPodY: Double,
    val strafePodX: Double,

    val forwardDirection: Encoder.Direction,
    val strafeDirection: Encoder.Direction,

    val odometerData: Odometer,
    val imuParameters: IMU.Parameters
)