package org.firstinspires.ftc.teamcode.following

class PoseLQRController(
    qPositionForward: Double,
    qVelocityForward: Double,
    qPositionStrafe: Double,
    qVelocityStrafe: Double,
    qPositionHeading: Double,
    qVelocityHeading: Double,
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
     * @param qPositionStrafe strafe position error weight
     * @param qVelocityStrafe strafe velocity error weight
     * @param qPositionHeading heading error weight
     * @param qVelocityHeading angular velocity error weight
     */
    init {
        k1Forward = kotlin.math.sqrt(qPositionForward)
        k2Forward = kotlin.math.sqrt(2.0 * k1Forward + qVelocityForward)

        k1Strafe = kotlin.math.sqrt(qPositionStrafe)
        k2Strafe = kotlin.math.sqrt(2.0 * k1Strafe + qVelocityStrafe)

        k1Heading = kotlin.math.sqrt(qPositionHeading)
        k2Heading = kotlin.math.sqrt(2.0 * k1Heading + qVelocityHeading)
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
    fun correctHeading(headingError: Double, angularVelocity: Double): Double {
        return -k1Heading * headingError - k2Heading * angularVelocity
    }
}
