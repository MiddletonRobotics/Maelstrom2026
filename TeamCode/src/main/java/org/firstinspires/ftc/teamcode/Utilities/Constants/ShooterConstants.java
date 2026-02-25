package org.firstinspires.ftc.teamcode.Utilities.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@Config
public class ShooterConstants {
    public static String shooterMotorID = "shooter";
    public static String hoodServoID = "hood";

    public static double knownDistance = 0;
    public static double knownArea = 0;
    public static double llAngle = 0;
    public static double targetHeight = 0;
    public static double shooterHeight = 0;
    public static double transferEfficiency = 0.8;
    public static double wheelRadius = 0;
    public static double closeVelocity = 1400;
    public static double farVelocity = 2800;
    public static double midVelocity = 1700;
    public static double velocityTolerance = 30;
    public static double speedDropThreshold = 150; // velocity drop (ticks/s) indicating ball launched

    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00037;

    // SOTM lead-time LUT: distance (inches) → lead time (seconds)
    public static double[] leadTimeDistances = { 20, 40, 50, 60, 70, 80, 90, 130, 140, 150 };
    public static double[] leadTimeValues = { 0.1, 0.4, 0.535, 0.555, 0.595, 0.74, 0.765, 0.85, 0.86, 0.9 };
}
