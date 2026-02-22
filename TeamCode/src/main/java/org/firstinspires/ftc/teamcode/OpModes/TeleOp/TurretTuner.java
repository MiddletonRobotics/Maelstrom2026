package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

@TeleOp(name="TurretTuner")
public class TurretTuner extends OpMode
{
    private Maelstrom robot;

    @Override
    public void init()
    {
        robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.BLUE,gamepad1,gamepad2);
        robot.turret.setPointMode();
    }

    public void loop()
    {
        robot.periodic();
        if(gamepad1.right_bumper)
        {
            robot.turret.setManualAngle(0);
        }
        if(gamepad1.left_bumper)
        {
            robot.turret.setManualAngle(70);
        }
        telemetry.update();
    }
}
