package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.blueReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.redReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.blueGoal;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.redGoal;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOPBindings;

@Config
public class Maelstrom extends Robot {
    public enum Alliance {
        RED, BLUE;
    }

    public Alliance color;
    public Drivetrain dt;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Vision cams;
    public GamepadEx driver1;
    public GamepadEx driver2;

    private TeleOPBindings mapping;
    private Telemetry telemetry;

    public static double blueCorner1x=99;
    public static double blueCorner1y=39;
    public static double blueCorner1H=135;
    public static Pose blueCorner1 = new Pose(blueCorner1x, blueCorner1y, Math.toRadians(blueCorner1H));
    private static Pose redCorner1 = blueCorner1.mirror();

    public static double blueCorner2x=113;
    public static double blueCorner2y=39;
    public static double blueCorner2H=45;
    public static Pose blueCorner2 = new Pose(blueCorner2x, blueCorner2y, Math.toRadians(blueCorner2H));
    private static Pose redCorner2 = blueCorner2.mirror();

    public static double blueCorner3x=100;
    public static double blueCorner3y=27;
    public static double blueCorner3H=225;
    public static Pose blueCorner3 = new Pose(blueCorner3x, blueCorner3y, Math.toRadians(blueCorner3H));
    private static Pose redCorner3 = blueCorner3.mirror();

    public static double blueCorner4x=112.5;
    public static double blueCorner4y=26.5;
    public static double blueCorner4H=315;
    public static Pose blueCorner4 = new Pose(blueCorner4x, blueCorner4y, Math.toRadians(blueCorner4H));
    private static Pose redCorner4 = blueCorner4.mirror();

    public Maelstrom(HardwareMap hMap, Telemetry telemetry, Alliance color, Gamepad d1, Gamepad d2) {
        cams = new Vision(hMap, telemetry, color);
        dt = new Drivetrain(hMap, telemetry, cams);
        this.telemetry = telemetry;
        intake = new Intake(hMap, telemetry);
        intake.stop();
        turret = new Turret(hMap, telemetry);
        shooter = new Shooter(hMap, telemetry, color, cams);
        driver1 = new GamepadEx(d1);
        driver2 = new GamepadEx(d2);
        this.color = color;
        mapping = new TeleOPBindings(driver1, driver2, this);
        register(dt, intake, shooter, turret, cams);
    }

    public void periodic() {
        dt.periodic();
        cams.periodic();
        intake.periodic();
        shooter.periodic();
        turret.periodic();
        turret.getTargetAngle(cams.getTargetX(), cams.targetPresent());
    }

    @Override
    public void reset() {
        intake.stop();
        shooter.flywheelOn = false;
        // turret.setTempOffset(turret.getAngle());
        Drivetrain.startPose = dt.follower.getPose();
    }

    public void initializeTeleOP() {
        mapping.controlMap();
        mapping.configureDefaultCommands();
        dt.enableTeleop();
    }
}
