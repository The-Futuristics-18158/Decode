package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.junit.Test;

/**
 * Debug test to isolate the odometry issue
 */
public class DebugOdometryTest {

    @Test
    public void debugOdometryChain() {
        System.out.println("=== Debug Odometry Chain ===");
        
        // Create simple kinematics
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );

        // Test 1: Check if kinematics work in isolation
        System.out.println("\n1. Testing Kinematics in Isolation:");
        ChassisSpeeds forwardSpeed = new ChassisSpeeds(1.0, 0.0, 0.0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(forwardSpeed);
        
        for (int i = 0; i < states.length; i++) {
            System.out.printf("Module %d: Speed=%.3f, Angle=%.3f rad (%.1f°)%n", 
                i, states[i].speedMetersPerSecond, states[i].angle.getRadians(),
                Math.toDegrees(states[i].angle.getRadians()));
        }

        // Test 2: Check reverse kinematics
        System.out.println("\n2. Testing Reverse Kinematics:");
        ChassisSpeeds recovered = kinematics.toChassisSpeeds(states);
        System.out.printf("Recovered: vx=%.3f, vy=%.3f, omega=%.3f%n", 
            recovered.vxMetersPerSecond, recovered.vyMetersPerSecond, recovered.omegaRadiansPerSecond);

        // Test 3: Check module position to state conversion
        System.out.println("\n3. Testing Module Position Delta Calculation:");
        SwerveModulePosition[] initialPos = {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
        };
        
        SwerveModulePosition[] newPos = {
            new SwerveModulePosition(0.1, new Rotation2d(0)),
            new SwerveModulePosition(0.1, new Rotation2d(0)),
            new SwerveModulePosition(0.1, new Rotation2d(0)),
            new SwerveModulePosition(0.1, new Rotation2d(0))
        };

        // Manually calculate deltas (what odometry should do internally)
        SwerveModuleState[] deltaStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double deltaDistance = newPos[i].distanceMeters - initialPos[i].distanceMeters;
            System.out.printf("Module %d delta distance: %.3f%n", i, deltaDistance);
            deltaStates[i] = new SwerveModuleState(deltaDistance, newPos[i].angle);
        }

        // Test 4: Convert deltas to chassis speeds
        System.out.println("\n4. Testing Delta to Chassis Speed Conversion:");
        ChassisSpeeds deltaChassisSpeed = kinematics.toChassisSpeeds(deltaStates);
        System.out.printf("Delta Chassis Speed: vx=%.3f, vy=%.3f, omega=%.3f%n", 
            deltaChassisSpeed.vxMetersPerSecond, deltaChassisSpeed.vyMetersPerSecond, 
            deltaChassisSpeed.omegaRadiansPerSecond);

        // Test 5: Create actual odometry and test
        System.out.println("\n5. Testing Full Odometry Update:");
        SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics, new Rotation2d(0), initialPos, new Pose2d());
        
        System.out.printf("Initial pose: (%.3f, %.3f, %.1f°)%n", 
            odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(),
            Math.toDegrees(odometry.getPoseMeters().getRotation().getRadians()));

        Pose2d updatedPose = odometry.update(new Rotation2d(0), newPos);
        
        System.out.printf("Updated pose: (%.3f, %.3f, %.1f°)%n", 
            updatedPose.getX(), updatedPose.getY(),
            Math.toDegrees(updatedPose.getRotation().getRadians()));
    }
}