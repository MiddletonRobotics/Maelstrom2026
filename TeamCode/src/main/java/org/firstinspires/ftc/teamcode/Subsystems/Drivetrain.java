package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.blueReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.measurementNoise;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.measurementNoise2;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.processNoise;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.processNoise2;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.redReset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants.startPose;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.blueGoal;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.redGoal;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utilities.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Utilities.Constants.GlobalConstants;
import org.firstinspires.ftc.teamcode.Utilities.Constants.GlobalConstants.OpModeType;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
    // private Motor leftFront;
    // private Motor rightFront;
    // private Motor leftRear;
    // private Motor rightRear;

    public Follower follower;
    private com.qualcomm.robotcore.hardware.CRServo park;
    private Vision cams;
    public static Pose startPose = new Pose(0, 0, 0);
    private Pose megaTagPose = new Pose(0, 0, 0);
    private Pose fusedPose= new Pose(0,0,0);

    private List<LynxModule> allHubs;

    public double distance=0;

    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;

    private Telemetry telemetry;

    public Drivetrain(HardwareMap aHardwareMap, Telemetry telemetry, Vision cam) {
      /*  leftFront = new Motor(aHardwareMap, DrivetrainConstants.fLMotorID, Motor.GoBILDA.RPM_312);
        rightFront = new Motor(aHardwareMap, DrivetrainConstants.fRMotorID, Motor.GoBILDA.RPM_312);
        leftRear = new Motor(aHardwareMap, DrivetrainConstants.bLMotorID, Motor.GoBILDA.RPM_312);
        rightRear = new Motor(aHardwareMap, DrivetrainConstants.bRMotorID, Motor.GoBILDA.RPM_312);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftRear.setRunMode(Motor.RunMode.RawPower);
        rightRear.setRunMode(Motor.RunMode.RawPower);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftRear.setInverted(false);
        rightRear.setInverted(false); */


        follower = Constants.createFollower(aHardwareMap);
        cams = cam;
        megaTagPose = cam.getPedro();
        park = aHardwareMap.get(CRServo.class, "park");
        this.telemetry = telemetry;
        xFilter = new KalmanFilter(processNoise, measurementNoise);
        yFilter = new KalmanFilter(processNoise2, measurementNoise2);

        allHubs=aHardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void periodic() {
        long startTime = System.nanoTime();
//        follower.updatePose();
        xFilter.setProcessNoise(processNoise);
        xFilter.setMeasurementNoise(measurementNoise);
        yFilter.setProcessNoise(processNoise2);
        yFilter.setMeasurementNoise(measurementNoise2);
        follower.update();
        cams.setMT2Orientation(follower.getPose());
        megaTagPose = cams.getPedro();
        fusedPose= getFusedPose(follower.getPose(), megaTagPose);
        //follower.setPose(fusedPose);
        telemetry.addData("Drivetrain Pose X", follower.getPose().getX());
        telemetry.addData("Drivetrain Pose Y", follower.getPose().getY());
        telemetry.addData("Drivetrain Heading", follower.getPose().getHeading());
        telemetry.addData("Fused X: ", fusedPose.getX());
        telemetry.addData("Fused Y: ", fusedPose.getY());
        telemetry.addData("Odo Distance: ", distance);
        long totalTime = System.nanoTime() - startTime;
        telemetry.addData("Loop Time: ", totalTime / 1000000);

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric) {

        follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric);
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric, double headingOffset) {
        follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric, headingOffset);
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric, Maelstrom.Alliance color) {
        if (color.equals(Maelstrom.Alliance.RED)) {
            follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric, 0);
        } else if (color.equals(Maelstrom.Alliance.BLUE)) {
            follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric, Math.toRadians(180));
        }

    }

    public void setMovementVectors(double strafe, double forward, double rotation) {
        follower.setTeleOpDrive(forward, -strafe, -rotation, false);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setStartingPose(Pose pose) {
        startPose = pose;
    }

        public Pose getFusedPose(Pose currentPose, Pose limelightPose) {
            double fusedX = xFilter.update(limelightPose.getX(), currentPose.getX());
            double fusedY = yFilter.update(limelightPose.getY(), currentPose.getY());
            double fusedHeading = currentPose.getHeading();

            return new Pose(fusedX, fusedY, fusedHeading);
        }

        public void enableTeleop () {
            follower.setStartingPose(startPose);
            follower.startTeleopDrive(true);
        }

        public void resetHeading (Maelstrom.Alliance color)
        {
            if (color.equals(Maelstrom.Alliance.RED)) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
            } else if (color.equals(Maelstrom.Alliance.BLUE)) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
            }
        }

        public void hold ()
        {
            follower.holdPoint(follower.getPose());
        }

        public double getHeading ()
        {
            return follower.getPose().getHeading();
        }

        public double getDistance()
        {
            return distance;
        }

        public double getX ()
        {
            return follower.getPose().getX();
        }

        public double getY ()
        {
            return follower.getPose().getY();
        }

        public Pose getPose ()
        {
            return follower.getPose();
        }

        public void parkUp ()
        {
            park.setPower(1);
        }
        public void parkDown()
        {
            park.setPower(-1);
        }
        public void stopPark()
        {
            park.setPower(0);
        }

        public void calcDistance(Maelstrom.Alliance color)
        {
            double x;
            double y;
            if(color.equals(Maelstrom.Alliance.BLUE)) {
                x = Math.pow(follower.getPose().getX() - blueGoal.getX(),2);
                y = Math.pow(follower.getPose().getY() - blueGoal.getY(),2);
            }
            else
            {
                x = Math.pow(follower.getPose().getX() - redGoal.getX(),2);
                y = Math.pow(follower.getPose().getY() - redGoal.getY(),2);
            }
            distance= Math.sqrt(x+y);
        }

        public void resetPose(Maelstrom.Alliance color)
        {
            if(color.equals(Maelstrom.Alliance.BLUE))
            {
                follower.setPose(blueReset);
            }
            else
            {
                follower.setPose(redReset);
            }
        }

        private static class KalmanFilter {
            private double processNoise;
            private double measurementNoise;
            private double estimate;
            private double errorCovariance;

            public KalmanFilter(double processNoise, double measurementNoise) {
                this.processNoise = processNoise;
                this.measurementNoise = measurementNoise;
                this.estimate = 0;
                this.errorCovariance = 1;
            }

            public double update(double measurement, double prediction) {
                errorCovariance += processNoise;
                double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
                estimate = prediction + kalmanGain * (measurement - prediction);
                errorCovariance = (1 - kalmanGain) * errorCovariance;
                return estimate;
            }

            public void setMeasurementNoise(double measurementNoise) {
                this.measurementNoise = measurementNoise;
            }

            public void setProcessNoise(double processNoise) {
                this.processNoise = processNoise;
            }
        }
    }


