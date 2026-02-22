package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{
    public enum intakeState{
        SPININ,SPINOUT,KICKING
    }
    private DcMotorEx intakeMotor;
    private Servo kicker;
    private Servo kicker2;
    //private RevColorSensorV3 sensor1;
    private RevColorSensorV3 sensor2;
    private NormalizedRGBA colors1;
    private NormalizedRGBA colors2;
    private Motor.Encoder intakeEncoder;
    private Telemetry telemetry;
    private int loopCounter=0;

    public Intake(HardwareMap aHardwareMap, Telemetry telemetry)
    {
        this.telemetry=telemetry;
        kicker= aHardwareMap.get(Servo.class,"kicker");
        kicker.setDirection(Servo.Direction.FORWARD);
        kicker2= aHardwareMap.get(Servo.class,"kicker2");
        kicker2.setDirection(Servo.Direction.REVERSE);
        intakeMotor= aHardwareMap.get(DcMotorEx.class,"intake");
        //sensor1= aHardwareMap.get(RevColorSensorV3.class,"sensor1");
        sensor2= aHardwareMap.get(RevColorSensorV3.class,"sensor2");
        //colors1=sensor1.getNormalizedColors();
        colors2=sensor2.getNormalizedColors();
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kicker.setPosition(0.6);
        kicker2.setPosition(0);
        stop();
    }

    @Override
    public void periodic()
    {
        //colors1=sensor1.getNormalizedColors();
        //colors2=sensor2.getNormalizedColors();

        //telemetry.addData("Sensor 1 distance: ", sensor1.getDistance(DistanceUnit.INCH));

        telemetry.addData("Sensor 2 Distance: ", sensor2.getDistance(DistanceUnit.INCH));

        //telemetry.addData("Sensor 1 Color: ",colors1.red + " " + colors1.green + " " + colors1.blue );
        //telemetry.addData("Sensor 2 Color: ",colors2.red + " " + colors2.green + " " + colors2.blue );
        loopCounter++;
    }

    public void setPower(double p)
    {
        intakeMotor.setPower(p);
    }

    public void spinIn()
    {
            intakeMotor.setPower(1);
    }

    public void idle()
    {
        intakeMotor.setPower(0.5);
    }
    public void slowSpinOut()
    {
        intakeMotor.setPower(-0.8);
    }
    public void spinOut()
    {
        intakeMotor.setPower(-1);
    }
    public void stop()
    {
        intakeMotor.setPower(0);
    }

    public boolean ballReady()
    {
        return !(sensor2.getDistance(DistanceUnit.INCH)<6);
    }

    public void kickerDown()
    {
        kicker.setPosition(0.75);
    }

    public void kickerUp()
    {
        kicker.setPosition(0.1);
    }

    public void kickerHalfway()
    {
        kicker.setPosition(0.15);
    }

    public void kicker2Up()
    {
        kicker2.setPosition(0.3);
    }

    public void kicker2down()
    {
        kicker2.setPosition(0);
    }

}
