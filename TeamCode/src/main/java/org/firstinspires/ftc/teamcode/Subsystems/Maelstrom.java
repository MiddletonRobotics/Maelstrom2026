package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.blueReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.redReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.blueGoal;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.redGoal;

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


public class Maelstrom extends Robot
{
    public enum Alliance{
        RED,BLUE;
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

    public static Pose blueCorner1= new Pose(99,40,Math.toRadians(135));
    private static Pose redCorner1= blueCorner1.mirror();

    public static Pose blueCorner2= new Pose(112,40,Math.toRadians(45));
    private static Pose redCorner2= blueCorner2.mirror();

    public static Pose blueCorner3= new Pose(99,26,Math.toRadians(225));
    private static Pose redCorner3= blueCorner3.mirror();

    public static Pose blueCorner4= new Pose(112,26,Math.toRadians(315));
    private static Pose redCorner4= blueCorner4.mirror();

    public Maelstrom(HardwareMap hMap, Telemetry telemetry, Alliance color, Gamepad d1, Gamepad d2)
    {
        cams= new Vision(hMap,telemetry,color);
        dt= new Drivetrain(hMap,telemetry,cams);
        this.telemetry=telemetry;
        intake= new Intake(hMap,telemetry);
        intake.stop();
        turret= new Turret(hMap,telemetry);
        shooter= new Shooter(hMap,telemetry,color,cams);
        driver1= new GamepadEx(d1);
        driver2= new GamepadEx(d2);
        this.color=color;
        mapping= new TeleOPBindings(driver1,driver2,this);
        register(dt,intake,shooter,turret,cams);
    }

    public void periodic()
    {
        dt.periodic();
        cams.periodic();
        shooter.updateDistance(dt.distance);
        intake.periodic();
        shooter.periodic();
        turret.periodic();
        turret.getTargetAngle(cams.getTargetX(),cams.targetPresent());
        dt.calcDistance(color);
    }

    @Override
    public void reset()
    {
        intake.stop();
        shooter.flywheelOn=false;
        //turret.setTempOffset(turret.getAngle());
        Drivetrain.startPose=dt.follower.getPose();
    }

    public void initializeTeleOP()
    {
        mapping.controlMap();
        mapping.configureDefaultCommands();
        dt.enableTeleop();
    }
}
