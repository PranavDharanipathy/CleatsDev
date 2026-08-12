package org.firstinspires.ftc.teamcode.localization.localizers

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.localization.OdometryPod

data class PinpointAttributes(

    val deviceName: String,

    val forwardPodOffsetMM: Double,
    val strafePodOffsetMM: Double,

    val forwardPodDirection: GoBildaPinpointDriver.EncoderDirection,
    val strafePodDirection: GoBildaPinpointDriver.EncoderDirection,

    val odometryPodData: OdometryPod
)