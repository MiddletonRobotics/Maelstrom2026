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
    private Telemetry telemetry;
    private int transferState=-1;
    private int closeState=-1;
    private Timer tTimer;
    private Timer cTimer;
    //private PathChain redPark;
    //private PathChain bluePark;
    private static Pose blueCorner1= new Pose(99,40,Math.toRadians(135));
    private static Pose redCorner1= blueCorner1.mirror();

    private static Pose blueCorner2= new Pose(112,40,Math.toRadians(45));
    private static Pose redCorner2= blueCorner2.mirror();

    private static Pose blueCorner3= new Pose(99,26,Math.toRadians(225));
    private static Pose redCorner3= blueCorner3.mirror();

    private static Pose blueCorner4= new Pose(112,26,Math.toRadians(315));
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
        tTimer=new Timer();
        cTimer= new Timer();
        this.color=color;
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
        aimTurretWithPose();
        farTransferAndShoot();
        closeTransferAndShoot();
        dt.calcDistance(color);
        telemetry.addData("Close Transfer State: ", closeState);
        telemetry.addData("Far Transfer State: ", transferState);
    }

    @Override
    public void reset()
    {
        intake.stop();
        shooter.flywheelOn=false;
        //turret.setTempOffset(turret.getAngle());
        Drivetrain.startPose=dt.follower.getPose();
    }

    public void controlMap()
    {
        dt.setMovementVectors(driver1.getLeftX(), driver1.getLeftY(), driver1.getRightX(), true,color);

        if(Math.abs(driver1.getLeftX())>0 || Math.abs(driver1.getLeftY())>0 || Math.abs(driver1.getRightX())>0)
        {
            //dt.follower.breakFollowing();
        }

        if(driver1.getButton(GamepadKeys.Button.TOUCHPAD))
        {
            dt.resetHeading(color);
        }
        if(driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
        {
            resetPose();
        }

        //turret.turretWithManualLimits(-driver2.getLeftX());

        if(driver2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
        {
            startCloseTransfer();
        }
        if(driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
        {
            startCloseTransfer();
        }
        if(driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
        {
            cancelTransfer();
        }


        if(driver1.getButton(GamepadKeys.Button.LEFT_BUMPER))
        {

        }

        if(driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
        {
            shooter.flywheelOn=true;
        }
        if(driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
        {
            shooter.flywheelOn=false;
        }

        if(driver2.getButton(GamepadKeys.Button.Y))
        {
            shooter.shootFar();
        }

        if(driver2.getButton(GamepadKeys.Button.B))
        {
            shooter.shootClose();
        }
        if(driver2.getButton(GamepadKeys.Button.X))
        {
            shooter.shootMid();
        }

        if(transferState==-1 && closeState==-1) {
            if (driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
            {
                intake.spinOut();
            }
            else if (driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
            {
                intake.spinIn();
            }
            else
            {
                intake.stop();
            }

            if (driver2.getButton(GamepadKeys.Button.DPAD_UP))
            {
                intake.kickerUp();
            }
            else if (!driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
            {
                intake.kickerDown();
            }

            if (driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
            {
                intake.kickerHalfway();
            } 
            else if (!driver2.getButton(GamepadKeys.Button.DPAD_UP))
            {
                intake.kickerDown();
            }

            if(driver2.getButton(GamepadKeys.Button.DPAD_LEFT))
            {
                intake.kicker2Up();
            }
            else
            {
                intake.kicker2down();
            }
        }

        if(!shooter.flywheelOn)
        {
            if(driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
            {
                shooter.reverseWheel();
            }
            else
            {
                shooter.stopFlywheel();
            }
        }

        if(driver1.getButton(GamepadKeys.Button.DPAD_UP))
        {
            park1();
        }
        if(driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
        {
            park2();
        }
        if(driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
        {
            park3();
        }
        if(driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
        {
            park4();
        }

        /*
        if(driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
        {
            shooter.setHoodServo(0);
        }
        if(driver1.getButton(GamepadKeys.Button.DPAD_UP))
        {
            shooter.setHoodServo(0.96);
        }

        if(driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
        {
            shooter.shootAutoVelocity();
        }
         */

        if(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5)
        {
            shooter.shootAutoVelocity();
        }
        if(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5)
        {
            shooter.useAuto=false;
        }
        if(driver1.getButton(GamepadKeys.Button.A))
        {
            dt.parkUp();
        }
        else if(driver1.getButton(GamepadKeys.Button.B))
        {
            dt.parkDown();
        }
        else {
            dt.stopPark();
        }

        if(driver1.getButton(GamepadKeys.Button.X))
        {
            turret.setManualAngle(0);
            turret.setPointMode();
        }
        if(driver1.getButton(GamepadKeys.Button.Y))
        {
            turret.startPoseTracking();
        }

        if(driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
        {
            turret.setManualAngle(-90);
        }
    }

    private void setState(int x)
    {
        transferState=x;
        tTimer.resetTimer();
    }

    private void setCloseState(int x)
    {
        closeState=x;
        cTimer.resetTimer();
    }
    private void startTransfer()
    {
        setState(1);
    }
    private void startCloseTransfer()
    {
        setCloseState(1);
    }
    private void cancelTransfer()
    {
        setState(-1);
        setCloseState(-1);
    }

    private void closeTransferAndShoot()
    {
        switch(closeState)
        {
            case 1:
                if(cTimer.getElapsedTimeSeconds()>0.1)
                {
                    intake.kickerUp();
                    setCloseState(2);
                }
                break;
            case 2:
                if(cTimer.getElapsedTimeSeconds()>0.15)
                {
                    intake.setPower(1);
                    setCloseState(3);
                }
                break;
            case 3:
                if(cTimer.getElapsedTimeSeconds()>.8)
                {
                    setCloseState(4);
                }
                break;
            case 4:
                if(cTimer.getElapsedTimeSeconds()>.3 || intake.ballReady())
                {
                    setCloseState(5);
                }
                break;
            case 5:
                if(cTimer.getElapsedTimeSeconds()>0.1)
                {
                    intake.kicker2Up();
                    setCloseState(6);
                }
                break;
            case 6:
                if(cTimer.getElapsedTimeSeconds()>0.35)
                {
                    intake.kicker2down();
                    intake.stop();
                    intake.kickerDown();
                    setCloseState(-1);
                }
                break;
        }
    }

    private void farTransferAndShoot()
    {
        switch(transferState)
        {
            case 1:
                if(tTimer.getElapsedTimeSeconds()>0.1)
                {
                    intake.kickerUp();
                    setState(2);
                }
                break;
            case 2:
                if(tTimer.getElapsedTimeSeconds()>0.15)
                {
                    intake.setPower(0.7);
                    setState(3);
                }
                break;
            case 3:
                if(tTimer.getElapsedTimeSeconds()>.8)
                {
                    setState(4);
                }
                break;
            case 4:
                if(tTimer.getElapsedTimeSeconds()>.3 || intake.ballReady())
                {
                    setState(5);
                }
                break;
            case 5:
                if(tTimer.getElapsedTimeSeconds()>0.1)
                {
                    intake.kicker2Up();
                    setState(6);
                }
                break;
            case 6:
                if(tTimer.getElapsedTimeSeconds()>0.25)
                {
                    intake.kicker2down();
                    intake.stop();
                    intake.kickerDown();
                    setState(-1);
                }
                break;
        }
    }

    private void park1()
    {
        if(color.equals(Alliance.BLUE))
        {
            PathChain bluePark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),blueCorner1)).setLinearHeadingInterpolation(dt.follower.getHeading(), blueCorner1.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(bluePark,true);
            }
        }
        else if(color.equals(Alliance.RED))
        {
            PathChain redPark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),redCorner1)).setLinearHeadingInterpolation(dt.follower.getHeading(), redCorner1.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(redPark,true);
            }
        }

    }

    private void park2()
    {
        if(color.equals(Alliance.BLUE))
        {
            PathChain bluePark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),blueCorner2)).setLinearHeadingInterpolation(dt.follower.getHeading(), blueCorner2.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(bluePark,true);
            }
        }
        else if(color.equals(Alliance.RED))
        {
            PathChain redPark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),redCorner2)).setLinearHeadingInterpolation(dt.follower.getHeading(), redCorner2.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(redPark,true);
            }
        }

    }

    private void park3()
    {
        if(color.equals(Alliance.BLUE))
        {
            PathChain bluePark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),blueCorner3)).setLinearHeadingInterpolation(dt.follower.getHeading(), blueCorner3.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(bluePark,true);
            }
        }
        else if(color.equals(Alliance.RED))
        {
            PathChain redPark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),redCorner3)).setLinearHeadingInterpolation(dt.follower.getHeading(), redCorner3.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(redPark,true);
            }
        }

    }

    private void park4()
    {
        if(color.equals(Alliance.BLUE))
        {
            PathChain bluePark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),blueCorner4)).setLinearHeadingInterpolation(dt.follower.getHeading(), blueCorner4.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(bluePark,true);
            }
        }
        else if(color.equals(Alliance.RED))
        {
            PathChain redPark= dt.follower.pathBuilder().addPath(new BezierLine(dt.follower.getPose(),redCorner4)).setLinearHeadingInterpolation(dt.follower.getHeading(), redCorner4.getHeading()).build();
            if(!dt.follower.isBusy())
            {
                dt.follower.followPath(redPark,true);
            }
        }

    }

    private void aimTurretWithPose()
    {
        if(color.equals(Alliance.BLUE))
        {
            turret.calculatePoseAngle(blueGoal,dt.getPose());
        }
        else if(color.equals(Alliance.RED))
        {
            turret.calculatePoseAngle(redGoal,dt.getPose());
        }
    }

    private void resetPose()
    {
        if(color.equals(Alliance.BLUE))
        {
            dt.setPose(blueReset);
        }
        else if(color.equals(Alliance.RED))
        {
            dt.setPose(redReset);
        }
    }
}
