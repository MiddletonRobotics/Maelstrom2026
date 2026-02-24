package org.firstinspires.ftc.teamcode.Utilities.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class TurretConstants {
    public static int startingPos = 0;
    public static int maxPos = 0;
    public static int minLimit = 0;
    public static int maxLimit = 2800;

    public static double kP = 0.025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double sP = 0.04; // 0.04;
    public static double sI = 0;
    public static double sD = 0.002;
    public static double sF = 0;

    public static double fS = 0.1;
    public static double kV = 0.0;
    public static double kA = 0;

    public static double kS = 0.3;
    public static double turretVelocityTolerance = 100;
    public static double max = 0;

    public static double PIDFSwitch = 10;

    public static double angleTolerance = 2;

    public static Pose blueGoal = new Pose(9, 138);
    public static Pose redGoal = blueGoal.mirror();

    // SOTM: minimum speed (in/s) before compensation activates
    public static double velocityThreshold = 1.0;

    // SOTM: compensated goals (updated every loop by Drivetrain)
    public static Pose compensatedBlueGoal = blueGoal;
    public static Pose compensatedRedGoal = redGoal;
}
