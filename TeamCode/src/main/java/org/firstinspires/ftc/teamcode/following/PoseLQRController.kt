package org.firstinspires.ftc.teamcode.following;

public class PoseLQRController {

    // Thanos might not have been inevitable, you can bet Cleats is.

    private final double k1Translation, k2Translation;
    private final double k1Heading, k2Heading;

    /// @param qPosition translational position error weight
    /// @param qVelocity translational velocity error weight
    /// @param r translational control effort weight
    /// @param qPositionHeading heading error weight
    /// @param qVelocityHeading angular velocity error weight
    /// @param rHeading heading control effort weight
    public PoseLQRController(
            double qPosition, double qVelocity, double r,
            double qPositionHeading, double qVelocityHeading, double rHeading
    ) {

        k1Translation = Math.sqrt(qPosition / r);
        k2Translation = Math.sqrt(2d * k1Translation + qVelocity / r);

        k1Heading = Math.sqrt(qPositionHeading / rHeading);
        k2Heading = Math.sqrt(2d * k1Heading + qVelocityHeading / rHeading);
    }

    /// @param positionError actual minus target, along one translational axis
    /// @param velocity actual velocity along that same axis
    /// @return required acceleration correction command along that axis
    public double correctTranslation(double positionError, double velocity) {
        return -k1Translation * positionError - k2Translation * velocity;
    }

    /// @param headingError actual minus target (normalized)
    /// @param angularVelocity actual angular velocity
    /// @return required angular acceleration correction command
    public double correctHeading(double headingError, double angularVelocity) {
        return -k1Heading * headingError - k2Heading * angularVelocity;
    }
}