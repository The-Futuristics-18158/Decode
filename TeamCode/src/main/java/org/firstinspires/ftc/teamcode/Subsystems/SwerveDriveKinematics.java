package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * module states (speed and angle).
 * 
 * The inverse kinematics (converting from a desired chassis velocity to individual module states) 
 * uses the relative locations of the modules with respect to the center of rotation.
 * 
 * The forward kinematics (converting from individual module states to a chassis velocity) is
 * performed by the signed area method described in Ether's paper.
 * 
 * Ported from WPILib's SwerveDriveKinematics for FTC use.
 */
public class SwerveDriveKinematics {
    private final int m_numModules;
    private final Translation2d[] m_modules;
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;
    
    private Translation2d m_prevCoR = new Translation2d();

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable number of module
     * locations as Translation2ds. The order in which you pass in the module locations is the same
     * order that you will receive the module states when performing inverse kinematics. It is also
     * expected that you pass in the module states in the same order when calling the forward
     * kinematics methods.
     *
     * @param moduleTranslationsMeters The locations of the modules relative to the physical center
     *     of the robot.
     */
    public SwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
        if (moduleTranslationsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        
        m_numModules = moduleTranslationsMeters.length;
        m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);
        
        for (int i = 0; i < m_numModules; i++) {
            // Inverse kinematics matrix setup
            // For each module, we need two rows:
            // Row 2*i:   [0, 1, +moduleX]  (x-component of module velocity - was y)  
            // Row 2*i+1: [1, 0, -moduleY]  (y-component of module velocity - was x)
            m_inverseKinematics.setRow(
                i * 2 + 0,
                0, /* vx */
                1, /* vy */
                +m_modules[i].getX() /* omega * +x */
            );
            m_inverseKinematics.setRow(
                i * 2 + 1,
                1, /* vx */
                0, /* vy */
                -m_modules[i].getY() /* omega * -y */
            );
        }
        
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will remain
     * stationary), the previously calculated module angle will be maintained.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)}
     *     function to rectify this issue.
     */
    public SwerveModuleState[] toSwerveModuleStates(
        ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

        if (chassisSpeeds.vxMetersPerSecond == 0.0
            && chassisSpeeds.vyMetersPerSecond == 0.0
            && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < m_numModules; i++) {
                moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
            }
            return moduleStates;
        }

        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.setRow(
                    i * 2 + 0,
                    1, /* vx */
                    0, /* vy */
                    -m_modules[i].getY() + centerOfRotationMeters.getY() /* omega * (-y + cor_y) */
                );
                m_inverseKinematics.setRow(
                    i * 2 + 1,
                    0, /* vx */
                    1, /* vy */
                    +m_modules[i].getX() - centerOfRotationMeters.getX() /* omega * (+x - cor_x) */
                );
            }
            m_prevCoR = centerOfRotationMeters;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
            0,
            0,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);

        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.sqrt(x * x + y * y);
            Rotation2d angle = new Rotation2d(Math.atan2(y, x));

            moduleStates[i] = new SwerveModuleState(speed, angle);
        }

        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis speed from the given module
     * states. This method is often used for odometry -- determining the robot's position on the
     * field using data from the real-world speed and angle of each module on the robot.
     *
     * @param moduleStates The state of the modules (as an array of SwerveModuleState types) as
     *     measured from respective encoders and gyros. The order of the module states should be same
     *     as passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        if (moduleStates.length != m_numModules) {
            throw new IllegalArgumentException(
                "Number of modules is not consistent with number of module states provided.");
        }

        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond * module.angle.getSin());
        }

        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
            chassisSpeedsVector.get(0, 0),
            chassisSpeedsVector.get(1, 0),
            chassisSpeedsVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one
     * can reduce all the wheel speeds to make sure that all requested module speeds are at-or-below
     * the absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
        SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                    moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    /**
     * Returns the module translations.
     *
     * @return The module translations.
     */
    public Translation2d[] getModuleTranslations() {
        return Arrays.copyOf(m_modules, m_modules.length);
    }
}