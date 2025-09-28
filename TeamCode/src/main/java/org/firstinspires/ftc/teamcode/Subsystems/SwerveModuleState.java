package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * Represents the state of a swerve module.
 * 
 * This class contains the velocity of the drive wheel and the angle of the module
 * as measured by the steering encoder. This is used for swerve drive kinematics
 * and control calculations.
 *
 * Ported from WPILib's SwerveModuleState for FTC use.
 */
public class SwerveModuleState {
    /** Speed of the wheel of the module in meters per second. */
    public double speedMetersPerSecond;

    /** Angle of the module. */
    public Rotation2d angle = new Rotation2d();

    /** Constructs a SwerveModuleState with zeros for speed and angle. */
    public SwerveModuleState() {}

    /**
     * Constructs a SwerveModuleState.
     *
     * @param speedMetersPerSecond The speed of the module in meters per second.
     * @param angle The angle of the module.
     */
    public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
    }

    /**
     * Compares two swerve module states for equality.
     *
     * @param other Object to compare against.
     * @return Whether the two swerve module states are equal or not.
     */
    @Override
    public boolean equals(Object other) {
        if (other instanceof SwerveModuleState) {
            SwerveModuleState otherState = (SwerveModuleState) other;
            return Math.abs(speedMetersPerSecond - otherState.speedMetersPerSecond) < 1e-9
                && angle.equals(otherState.angle);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(speedMetersPerSecond) ^ angle.hashCode();
    }

    /**
     * Returns a copy of this swerve module state.
     *
     * @return A copy.
     */
    public SwerveModuleState copy() {
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    @Override
    public String toString() {
        return String.format(
            "SwerveModuleState(Speed: %.2f m/s, Angle: %s)",
            speedMetersPerSecond, angle);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * setContinuous functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
        
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getRadians(), 
                                                          desiredState.angle.getRadians());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getRadians();
        
        if (Math.abs(delta) > Math.PI / 2) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
        }
        
        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
    }

    /**
     * Places newAngle in the same 0 to 360 scope as scopeReference.
     *
     * @param scopeReference Angle to use as reference for scope.
     * @param newAngle Angle to place in scope.
     * @return newAngle in the same scope as scopeReference.
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % (2 * Math.PI);
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (2 * Math.PI - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (2 * Math.PI + lowerOffset);
        }
        
        while (newAngle < lowerBound) {
            newAngle += 2 * Math.PI;
        }
        while (newAngle > upperBound) {
            newAngle -= 2 * Math.PI;
        }
        if (newAngle - scopeReference > Math.PI) {
            newAngle -= 2 * Math.PI;
        } else if (newAngle - scopeReference < -Math.PI) {
            newAngle += 2 * Math.PI;
        }
        
        return newAngle;
    }

    /**
     * Subtracts other from the current state.
     *
     * @param other The state to subtract.
     * @return The difference between the two states.
     */
    public SwerveModuleState minus(SwerveModuleState other) {
        return new SwerveModuleState(
            speedMetersPerSecond - other.speedMetersPerSecond,
            angle.minus(other.angle));
    }

    /**
     * Adds other to the current state.
     *
     * @param other The state to add.
     * @return The sum of the two states.
     */
    public SwerveModuleState plus(SwerveModuleState other) {
        return new SwerveModuleState(
            speedMetersPerSecond + other.speedMetersPerSecond,
            angle.plus(other.angle));
    }

    /**
     * Multiplies the state by a scalar.
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled state.
     */
    public SwerveModuleState times(double scalar) {
        return new SwerveModuleState(
            speedMetersPerSecond * scalar,
            angle.times(scalar));
    }

    /**
     * Divides the state by a scalar.
     *
     * @param scalar The scalar to divide by.
     * @return The scaled state.
     */
    public SwerveModuleState div(double scalar) {
        return times(1.0 / scalar);
    }
}