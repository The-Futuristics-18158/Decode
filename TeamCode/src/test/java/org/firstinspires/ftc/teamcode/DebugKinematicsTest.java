package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.junit.Test;

/**
 * Simple debug test to isolate the kinematics issue
 */
public class DebugKinematicsTest {

    @Test
    public void debugKinematics() {
        System.out.println("=== Debug Kinematics Test ===");
        
        // Create a simple square robot
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.25, 0.25),   // Front Left
            new Translation2d(0.25, -0.25),  // Front Right
            new Translation2d(-0.25, 0.25),  // Back Left
            new Translation2d(-0.25, -0.25)  // Back Right
        );

        // Test pure forward motion
        ChassisSpeeds forwardMotion = new ChassisSpeeds(1.0, 0.0, 0.0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(forwardMotion);
        
        System.out.println("Input: vx=1.0, vy=0.0, omega=0.0");
        
        for (int i = 0; i < states.length; i++) {
            System.out.println("Module " + i + ":");
            System.out.println("  Speed: " + states[i].speedMetersPerSecond);
            System.out.println("  Angle (radians): " + states[i].angle.getRadians());
            System.out.println("  Angle (degrees): " + Math.toDegrees(states[i].angle.getRadians()));
        }
    }
}