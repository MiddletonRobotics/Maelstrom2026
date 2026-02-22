package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

@TeleOp(name="TurretTroubleshooting2")
public class TurretTroubleshooting2 extends OpMode
{
    private Maelstrom robot;

    @Override
    public void init()
    {
        robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.RED,gamepad1,gamepad2);
        robot.turret.stopAndReset();
        robot.turret.startPoseTracking();
    }

    @Override
    public void loop()
    {
        robot.periodic();
        telemetry.update();
    }
}
