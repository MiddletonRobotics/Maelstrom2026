package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.Storage;

@TeleOp(name="BlueCompetitionTeleOP")
public class BlueCompetitionTeleOP extends CommandOpMode
{
    private Maelstrom Robot;


    @Override
    public void initialize()
    {
        Robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.BLUE,gamepad1,gamepad2);
        telemetry.addData("tempOffset: ", Storage.turretOffset);
        Robot.turret.setManualAngle(0);
        Robot.initializeTeleOP();
        Robot.turret.updateOffset();
        Robot.turret.setPointMode();
    }

    @Override
    public void run()
    {
        super.run();
        telemetry.update();
    }
}
