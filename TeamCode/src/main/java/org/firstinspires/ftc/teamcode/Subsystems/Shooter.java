package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.closeVelocity;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.farVelocity;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.kP;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.midVelocity;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants.velocityTolerance;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants;

//import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;

public class Shooter extends SubsystemBase {
    public DcMotorEx shooterMotor;
    private Servo hoodServo;
    private double hoodAngle;
    private Servo light;
    private Vision cam;
    private InterpLUT table;
    private InterpLUT hoodTable;
    private Maelstrom.Alliance alliance;
    private Telemetry telemetry;
    public double currentVelocity;
    public double targetVelocity;
    public double autoVelocity;
    public boolean flywheelOn;
    public boolean useAuto = false;
    private double distance = 1;
    public PIDFController velocityController = new PIDFController(kP, kI, kD, ShooterConstants.kF);

    public Shooter(HardwareMap aHardwareMap, Telemetry telemetry, Maelstrom.Alliance color, Vision cam) {
        this.telemetry = telemetry;
        this.alliance = color;

        shooterMotor = aHardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, 0);

        hoodServo = aHardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        hoodServo.setPosition(0);

        light = aHardwareMap.get(Servo.class, "light");
        this.cam = cam;
        distance = 1;
        autoVelocity = 0;

        table = new InterpLUT();
        table.add(0, 1000);
        table.add(37, 1400);
        table.add(42.5, 1500);
        table.add(70.5, 1700);
        table.add(149, 2250);
        table.createLUT();

        hoodTable = new InterpLUT();
        hoodTable.add(1000, 0);
        hoodTable.add(1400, 0.1);
        hoodTable.add(1600, 0.45);
        hoodTable.add(2250, 0.86);
        hoodTable.createLUT();

        flywheelOn = false;
        targetVelocity = ShooterConstants.closeVelocity; // start with some default
    }

    @Override
    public void periodic() {
        updateAutoVelocity();
        autoHood();
        if (useAuto) {
            // shootAutoVelocity();
        }
        setHood(hoodAngle);

        // Update shooter velocity
        if (flywheelOn) {
            shooterMotor.setPower(velocityController.calculate(currentVelocity, targetVelocity));
        } else {
            shooterMotor.setPower(0);
        }

        currentVelocity = shooterMotor.getVelocity();

        // Keep all servo positions intact
        setLight();

        telemetry.addData("Flywheel On", flywheelOn);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Auto Velocity: ", autoVelocity);
        telemetry.addData("Use Auto: ", useAuto);
        // telemetry.addData("Auto Velocity",autoVelocity);
    }

    public void shootClose() {
        targetVelocity = ShooterConstants.closeVelocity;
    }

    public void shootMid() {
        targetVelocity = midVelocity;
    }

    public void shootFar() {
        targetVelocity = ShooterConstants.farVelocity;
    }

    public void shootFarAuto() {
        targetVelocity = 2000;
    }

    public void setTargetVelocity(double v) {
        targetVelocity = v;
    }

    public void shootAutoVelocity() {
        targetVelocity = autoVelocity;
    }

    public void reverseWheel() {
        shooterMotor.setPower(-0.1);
        flywheelOn = false;
    }

    public void toggleFlywheel() {
        flywheelOn = !flywheelOn;
    }

    public void stopFlywheel() {
        shooterMotor.setPower(0);
        flywheelOn = false;
    }

    public void enableFlywheel() {
        flywheelOn = true;
    }

    public void disableFlywheel() {
        flywheelOn = false;
    }

    public void setHoodServo(double angle) {
        hoodServo.setPosition(angle);
    }

    public void setFullPower() {
        shooterMotor.setPower(1);
    }

    public void setLight() {
        if (atSpeed()) {
            light.setPosition(0.5);
        } else {
            light.setPosition(0.277);
        }
    }

    public boolean atSpeed() {
        return Math.abs(currentVelocity - targetVelocity) < velocityTolerance;
    }

    /**
     * Returns true when flywheel speed drops substantially, indicating a ball was
     * launched.
     */
    public boolean speedDropped() {
        return (targetVelocity - currentVelocity) > ShooterConstants.speedDropThreshold;
    }

    public void updateDistance(double dist) {
        // distance= cam.getDistance();
        distance = Math.max(Math.min(dist, 148), 1);
    }

    public void updateAutoVelocity() {
        autoVelocity = table.get(distance);
    }

    public void hoodUp() {
        hoodServo.setPosition(0.96);
    }

    public void setHood(double d) {
        hoodServo.setPosition(d);
    }

    private void autoHood() {
        double vel = Math.max(Math.min(shooterMotor.getVelocity(), 2249), 1001);
        hoodAngle = hoodTable.get(vel);
    }

    public void updateCommand(double dist) {
        updateDistance(dist);
        shootAutoVelocity();
    }

    public void toggleAuto() {
        useAuto = !useAuto;
    }

}
