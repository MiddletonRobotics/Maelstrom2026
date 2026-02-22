package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.blueGoal;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.TurretConstants.redGoal;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.function.Supplier;

public class AutoAimTurret extends CommandBase
{
    private final Turret turret;
    private final Supplier<Pose> botPose;
    private final Maelstrom.Alliance color;

    public AutoAimTurret(Maelstrom robot, Supplier<Pose> robotPose)
    {
        this.turret=robot.turret;
        this.botPose=robotPose;
        this.color=robot.color;

        addRequirements(turret);
    }

    @Override
    public void execute()
    {
        if(color.equals(Maelstrom.Alliance.BLUE))
        {
            turret.calculatePoseAngle(blueGoal,botPose.get());
        }
        else if(color.equals(Maelstrom.Alliance.RED))
        {
            turret.calculatePoseAngle(redGoal,botPose.get());
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}
