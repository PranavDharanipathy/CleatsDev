package org.firstinspires.ftc.teamcode.following;

public class PrecisionModeThresholds {

    public final double entryPositionDistance, exitPositionDistance;
    public final double entryVelocity, exitVelocity;
    public final double entryHeadingError, exitHeadingError;
    public final double entryAngularVelocity, exitAngularVelocity;

    public PrecisionModeThresholds(
            double entryPositionDistance, double exitPositionDistance,
            double entryVelocity, double exitVelocity,
            double entryHeadingError, double exitHeadingError,
            double entryAngularVelocity, double exitAngularVelocity
    ) {

        this.entryPositionDistance = entryPositionDistance;
        this.exitPositionDistance = exitPositionDistance;

        this.entryVelocity = entryVelocity;
        this.exitVelocity = exitVelocity;

        this.entryHeadingError = entryHeadingError;
        this.exitHeadingError = exitHeadingError;

        this.entryAngularVelocity = entryAngularVelocity;
        this.exitAngularVelocity = exitAngularVelocity;
    }
}