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
    public static double farVelocity = 2200;
    public static double midVelocity = 1700;
    public static double velocityTolerance = 30;

    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00037;

    // SOTM lead-time LUT: distance (inches) → lead time (seconds)
    public static double[] leadTimeDistances = { 20, 50, 80, 110, 140 };
    public static double[] leadTimeValues = { 0.1, 0.2, 0.3, 0.4, 0.5 };
}
