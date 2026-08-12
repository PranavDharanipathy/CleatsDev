package org.firstinspires.ftc.teamcode.localization.localizers

import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.localization.Encoder
import org.firstinspires.ftc.teamcode.localization.OdometryPod

data class DriveEncoderIMUAttributes(
    
    val frontLeftName: String,
    val frontRightName: String,
    val backLeftName: String,
    val backRightName: String,
    val imuName: String,

    val frontLeftDirection: Encoder.Direction,
    val frontRightDirection: Encoder.Direction,
    val backLeftDirection: Encoder.Direction,
    val backRightDirection: Encoder.Direction,

    val chassisWidth: Double,
    val chassisLength: Double,

    val odometryData: OdometryPod,
    val imuParameters: IMU.Parameters
)