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
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants;

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
    private Pose fusedPose = new Pose(0, 0, 0);

    private List<LynxModule> allHubs;

    public static double distance = 0;
    public static double compensatedDistance = 0;
    private InterpLUT leadTimeLUT;

    private KalmanFilter xFilter;
    private KalmanFilter yFilter;
    private KalmanFilter headingFilter;

    KalmanFilterParameters filterParameters = new KalmanFilterParameters(
            0.01,  // modelCovariance: how much you trust your model (lower = trust model more)
            0.1    // dataCovariance: how noisy your vision data is (lower = trust vision more)
    );

    private Telemetry telemetry;

    public Drivetrain(HardwareMap aHardwareMap, Telemetry telemetry, Vision cam) {
        /*
         * leftFront = new Motor(aHardwareMap, DrivetrainConstants.fLMotorID,
         * Motor.GoBILDA.RPM_312);
         * rightFront = new Motor(aHardwareMap, DrivetrainConstants.fRMotorID,
         * Motor.GoBILDA.RPM_312);
         * leftRear = new Motor(aHardwareMap, DrivetrainConstants.bLMotorID,
         * Motor.GoBILDA.RPM_312);
         * rightRear = new Motor(aHardwareMap, DrivetrainConstants.bRMotorID,
         * Motor.GoBILDA.RPM_312);
         * 
         * leftFront.setRunMode(Motor.RunMode.RawPower);
         * rightFront.setRunMode(Motor.RunMode.RawPower);
         * leftRear.setRunMode(Motor.RunMode.RawPower);
         * rightRear.setRunMode(Motor.RunMode.RawPower);
         * 
         * leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
         * rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
         * leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
         * rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
         * 
         * leftFront.setInverted(true);
         * rightFront.setInverted(true);
         * leftRear.setInverted(false);
         * rightRear.setInverted(false);
         */

        xFilter = new KalmanFilter(filterParameters);
        yFilter = new KalmanFilter(filterParameters);
        headingFilter = new KalmanFilter(filterParameters);

        follower = Constants.createFollower(aHardwareMap);
        cams = cam;
        megaTagPose = cam.getPedro();
        park = aHardwareMap.get(CRServo.class, "park");
        this.telemetry = telemetry;

        allHubs = aHardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Build lead-time LUT for SOTM
        leadTimeLUT = new InterpLUT();
        for (int i = 0; i < ShooterConstants.leadTimeDistances.length; i++) {
            leadTimeLUT.add(ShooterConstants.leadTimeDistances[i],
                    ShooterConstants.leadTimeValues[i]);
        }
        leadTimeLUT.createLUT();
    }

    @Override
    public void periodic() {
        long startTime = System.nanoTime();
        // follower.updatePose();
        follower.update();
        cams.setMT2Orientation(follower.getPose());
        megaTagPose = cams.getPedro();
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

    public void updateWithVision(Pose estimatedVisionPose) {
        xFilter.update(0, estimatedVisionPose.getX());
        yFilter.update(0, estimatedVisionPose.getY());
        headingFilter.update(0, estimatedVisionPose.getHeading());

        // Get filtered values
        double filteredX = xFilter.getState();
        double filteredY = yFilter.getState();
        double filteredHeading = headingFilter.getState();

        // Fuse with odometry (weighted average or direct set)
        Pose2d filteredVisionPose = new Pose2d(filteredX, filteredY, new Rotation2d(filteredHeading));
        fusePoseEstimate(filteredVisionPose);
    }

    private void fusePoseEstimate(Pose2d visionPose) {
        Pose currentPose = getPose();

        double visionWeight = 0.3; // How much to trust vision vs odometry
        double fusedX = currentPose.getX() * (1 - visionWeight) + visionPose.getX() * visionWeight;
        double fusedY = currentPose.getY() * (1 - visionWeight) + visionPose.getY() * visionWeight;
        double fusedHeading = currentPose.getHeading() * (1 - visionWeight) + visionPose.getHeading() * visionWeight;

        setPose(new Pose(fusedX, fusedY, currentPose.getHeading()));
        // setPoseEstimate(visionPose); Test this if vision is goated or not
    }

    public void resetVisionFilters(double x, double y, double heading) {
        xFilter.reset(x, 0.1, 1.0);
        yFilter.reset(y, 0.1, 1.0);
        headingFilter.reset(heading, 0.1, 1.0);
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric) {

        follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric);
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric,
            double headingOffset) {
        follower.setTeleOpDrive(forward, -strafe, -rotation, !feildCentric, headingOffset);
    }

    public void setMovementVectors(double strafe, double forward, double rotation, boolean feildCentric,
            Maelstrom.Alliance color) {
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

    public void enableTeleop() {
        follower.setStartingPose(startPose);
        follower.startTeleopDrive(true);
    }

    public void resetHeading(Maelstrom.Alliance color) {
        if (color.equals(Maelstrom.Alliance.RED)) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
        } else if (color.equals(Maelstrom.Alliance.BLUE)) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
        }
    }

    public void hold() {
        follower.holdPoint(follower.getPose());
    }

    public double getHeading() {
        return follower.getPose().getHeading();
    }

    public double getDistance() {
        return distance;
    }

    public double getX() {
        return follower.getPose().getX();
    }

    public double getY() {
        return follower.getPose().getY();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void parkUp() {
        park.setPower(1);
    }

    public void parkDown() {
        park.setPower(-1);
    }

    public void stopPark() {
        park.setPower(0);
    }

    public void calcDistance(Maelstrom.Alliance color) {
        double x;
        double y;
        if (color.equals(Maelstrom.Alliance.BLUE)) {
            x = Math.pow(follower.getPose().getX() - blueGoal.getX(), 2);
            y = Math.pow(follower.getPose().getY() - blueGoal.getY(), 2);
        } else {
            x = Math.pow(follower.getPose().getX() - redGoal.getX(), 2);
            y = Math.pow(follower.getPose().getY() - redGoal.getY(), 2);
        }
        distance = Math.sqrt(x + y);
        compensatedDistance = distance;
    }

    public void resetPose(Maelstrom.Alliance color) {
        if (color.equals(Maelstrom.Alliance.BLUE)) {
            follower.setPose(blueReset);
        } else {
            follower.setPose(redReset);
        }
    }

    // ── SOTM velocity decomposition ──────────────────────────────

    /** Raw velocity vector from the follower (field frame). */
    public Vector getVelocityVector() {
        return follower.getVelocity();
    }

    /**
     * Velocity component along the robot→goal direction (positive = approaching).
     */
    public double getRadialVelocity(Pose goal) {
        double angleToGoal = Math.atan2(goal.getY() - getPose().getY(),
                goal.getX() - getPose().getX());
        Vector vel = getVelocityVector();
        return vel.getXComponent() * Math.cos(angleToGoal)
                + vel.getYComponent() * Math.sin(angleToGoal);
    }

    /** Velocity component perpendicular to robot→goal line (positive = left). */
    public double getTangentialVelocity(Pose goal) {
        double angleToGoal = Math.atan2(goal.getY() - getPose().getY(),
                goal.getX() - getPose().getX());
        Vector vel = getVelocityVector();
        return -vel.getXComponent() * Math.sin(angleToGoal)
                + vel.getYComponent() * Math.cos(angleToGoal);
    }

    /**
     * Computes compensated goal pose and distance, writes results to
     * TurretConstants.compensatedBlueGoal / compensatedRedGoal and
     * this.compensatedDistance.
     */
    public void updateCompensatedGoal(Maelstrom.Alliance color) {
        Pose staticGoal = color.equals(Maelstrom.Alliance.BLUE) ? blueGoal : redGoal;
        double speed = getVelocityVector().getMagnitude();

        if (speed < TurretConstants.velocityThreshold) {
            // Below threshold: no compensation
            TurretConstants.compensatedBlueGoal = blueGoal;
            TurretConstants.compensatedRedGoal = redGoal;
            compensatedDistance = distance;
            return;
        }

        double clampedDist = Math.max(Math.min(distance,
                ShooterConstants.leadTimeDistances[ShooterConstants.leadTimeDistances.length - 1]-1),
                ShooterConstants.leadTimeDistances[0]+1);
        double leadTime = leadTimeLUT.get(clampedDist);

        // Radial compensation: adjust effective distance
        double vRadial = getRadialVelocity(staticGoal);
        compensatedDistance = Math.max(distance - vRadial * leadTime, 1);

        // Tangential compensation: shift goal perpendicular to robot→goal line
        double vTangential = getTangentialVelocity(staticGoal);
        double angleToGoal = Math.atan2(staticGoal.getY() - getPose().getY(),
                staticGoal.getX() - getPose().getX());
        double perpX = -Math.sin(angleToGoal);
        double perpY = Math.cos(angleToGoal);

        Pose compensatedGoal = new Pose(
                staticGoal.getX() - perpX * vTangential * leadTime,
                staticGoal.getY() - perpY * vTangential * leadTime);

        if (color.equals(Maelstrom.Alliance.BLUE)) {
            TurretConstants.compensatedBlueGoal = compensatedGoal;
            TurretConstants.compensatedRedGoal = redGoal;
        } else {
            TurretConstants.compensatedRedGoal = compensatedGoal;
            TurretConstants.compensatedBlueGoal = blueGoal;
        }
    }
}
