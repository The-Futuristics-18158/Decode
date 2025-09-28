package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * Represents the position of a swerve module.
 * 
 * This class is used to represent the distance traveled by the drive motor and the angle
 * of the module (as determined by the steer angle encoder). This is used for swerve 
 * drive odometry calculations.
 *
 * Ported from WPILib's SwerveModulePosition for FTC use.
 */
public class SwerveModulePosition {
    /** Distance measured by the wheel of the module in meters. */
    public double distanceMeters;

    /** Angle of the module as measured by the angle encoder. */
    public Rotation2d angle = new Rotation2d();

    /** Constructs a SwerveModulePosition with zeros for distance and angle. */
    public SwerveModulePosition() {}

    /**
     * Constructs a SwerveModulePosition.
     *
     * @param distanceMeters The distance measured by the wheel of the module in meters.
     * @param angle The angle of the module.
     */
    public SwerveModulePosition(double distanceMeters, Rotation2d angle) {
        this.distanceMeters = distanceMeters;
        this.angle = angle;
    }

    /**
     * Compares two swerve module positions for equality.
     *
     * @param other Object to compare against.
     * @return Whether the two swerve module positions are equal or not.
     */
    @Override
    public boolean equals(Object other) {
        if (other instanceof SwerveModulePosition) {
            SwerveModulePosition otherPosition = (SwerveModulePosition) other;
            return Math.abs(distanceMeters - otherPosition.distanceMeters) < 1e-9
                && angle.equals(otherPosition.angle);
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(distanceMeters) ^ angle.hashCode();
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePosition copy() {
        return new SwerveModulePosition(distanceMeters, angle);
    }

    @Override
    public String toString() {
        return String.format(
            "SwerveModulePosition(Distance: %.2f m, Angle: %s)",
            distanceMeters, angle);
    }

    /**
     * Subtracts other from the current position.
     *
     * @param other The position to subtract.
     * @return The difference between the two positions.
     */
    public SwerveModulePosition minus(SwerveModulePosition other) {
        return new SwerveModulePosition(
            distanceMeters - other.distanceMeters,
            angle.minus(other.angle));
    }

    /**
     * Adds other to the current position.
     *
     * @param other The position to add.
     * @return The sum of the two positions.
     */
    public SwerveModulePosition plus(SwerveModulePosition other) {
        return new SwerveModulePosition(
            distanceMeters + other.distanceMeters,
            angle.plus(other.angle));
    }

    /**
     * Multiplies the position by a scalar.
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled position.
     */
    public SwerveModulePosition times(double scalar) {
        return new SwerveModulePosition(
            distanceMeters * scalar,
            angle.times(scalar));
    }

    /**
     * Divides the position by a scalar.
     *
     * @param scalar The scalar to divide by.
     * @return The scaled position.
     */
    public SwerveModulePosition div(double scalar) {
        return times(1.0 / scalar);
    }
}