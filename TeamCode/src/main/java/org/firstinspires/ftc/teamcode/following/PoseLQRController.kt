package org.firstinspires.ftc.teamcode.following

class PoseLQRController(
    qPositionForward: Double,
    qVelocityForward: Double,
    rForward: Double,
    qPositionStrafe: Double,
    qVelocityStrafe: Double,
    rStrafe: Double,
    qPositionHeading: Double,
    qVelocityHeading: Double,
    rHeading: Double
) {
    // Thanos might not have been inevitable, you can bet Cleats is.

    private val k1Forward: Double
    private val k2Forward: Double

    private val k1Strafe: Double
    private val k2Strafe: Double

    private val k1Heading: Double
    private val k2Heading: Double

    /**
     * Creates a pose LQR controller.
     *
     * @param qPositionForward forward position error weight
     * @param qVelocityForward forward velocity error weight
     * @param rForward forward control effort weight
     * @param qPositionStrafe strafe position error weight
     * @param qVelocityStrafe strafe velocity error weight
     * @param rStrafe strafe control effort weight
     * @param qPositionHeading heading error weight
     * @param qVelocityHeading angular velocity error weight
     * @param rHeading heading control effort weight
     */
    init {
        k1Forward = kotlin.math.sqrt(qPositionForward / rForward)
        k2Forward = kotlin.math.sqrt(
            2.0 * k1Forward + qVelocityForward / rForward
        )

        k1Strafe = kotlin.math.sqrt(qPositionStrafe / rStrafe)
        k2Strafe = kotlin.math.sqrt(
            2.0 * k1Strafe + qVelocityStrafe / rStrafe
        )

        k1Heading = kotlin.math.sqrt(qPositionHeading / rHeading)
        k2Heading = kotlin.math.sqrt(
            2.0 * k1Heading + qVelocityHeading / rHeading
        )
    }

    /**
     * @param positionError actual minus target, along the forward axis
     * @param velocity actual velocity along that same axis
     * @return required acceleration correction command along the axis
     */
    fun correctForward(positionError: Double, velocity: Double): Double {
        return -k1Forward * positionError - k2Forward * velocity
    }

    /**
     * @param positionError actual minus target, along the strafe axis
     * @param velocity actual velocity along that same axis
     * @return required acceleration correction command along the axis
     */
    fun correctStrafe(positionError: Double, velocity: Double): Double {
        return -k1Strafe * positionError - k2Strafe * velocity
    }

    /**
     * @param headingError actual minus target (normalized)
     * @param angularVelocity actual angular velocity
     * @return required angular acceleration correction command
     */
    fun correctHeading(
        headingError: Double,
        angularVelocity: Double
    ): Double {
        return -k1Heading * headingError - k2Heading * angularVelocity
    }
}
