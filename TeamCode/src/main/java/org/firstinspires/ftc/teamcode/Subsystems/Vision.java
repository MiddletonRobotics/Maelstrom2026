package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.Constants.VisionConstants.cameraPitch;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.VisionConstants.goalHeight;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Utilities.Constants.VisionConstants;

public class Vision extends SubsystemBase
{
    public Limelight3A cam;
    private List<LLResultTypes.FiducialResult> tagList;
    private Maelstrom.Alliance alliance;
    private Telemetry telemetry;
    private double targetx;
    public double distance=1;
    public Pose pedroPose= new Pose(0,0,0);

    public Vision(HardwareMap hMap, Telemetry telemetry, Maelstrom.Alliance color)
    {
        alliance=color;
        this.telemetry=telemetry;
        cam= hMap.get(Limelight3A.class,"limelight");

        // CRITICAL: Start the Limelight before trying to get results
        //cam.start();

        cam.setPollRateHz(25);
        
        // Initialize tagList as empty - it will be populated in periodic()
        tagList = new ArrayList<>();
    }


    @Override
    public void periodic()
    {
        // Always update tagList from the latest result, but check if result is valid first
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            tagList = result.getFiducialResults();
            
            // Update targetx if we have a valid tag
            LLResultTypes.FiducialResult tag = getTag();
            if (tag != null) {
                targetx = tag.getTargetXDegrees();
            }
        } else {
            // If result is invalid, clear the tag list
            tagList = new ArrayList<>();
        }
        calcDistance();
        getBotPose();

        telemetry.addData("Target X: ", targetx);
        telemetry.addData("target y; ",getTargetYDegrees());
        telemetry.addData("Target Present: ", targetPresent());
        telemetry.addData("Fiducial Count: ", tagList != null ? tagList.size() : 0);
        telemetry.addData("Distance: ",distance);
        telemetry.addData("Pedro X: ", pedroPose.getX());
        telemetry.addData("Pedro Y: ", pedroPose.getY());
        telemetry.addData("Pedro Heading: ", pedroPose.getHeading());
    }
    public LLResultTypes.FiducialResult getTag()
    {
        LLResultTypes.FiducialResult target= null;
        if(alliance.equals(Maelstrom.Alliance.BLUE))
        {
            if (tagList!=null)
            {
                for(LLResultTypes.FiducialResult tar:tagList)
                {
                    if(tar!=null && tar.getFiducialId() == 20)
                    {
                        target=tar;
                        break;
                    }
                }
            }
        }
        else if(alliance.equals(Maelstrom.Alliance.RED))
        {
            if(tagList!=null)
            {
                for(LLResultTypes.FiducialResult tar:tagList)
                {
                    if(tar!=null && tar.getFiducialId() == 24)
                    {
                        target=tar;
                        break;
                    }
                }
            }
        }
        return target;
    }

    public double getTargetX()
    {
        if(getTag()!=null) {
            return getTag().getTargetXDegrees();
        }
        return 0;
    }

    public boolean shouldTrustVision(Pose visionEstimate, Pose currentRobotPose) {
        double deltaX = Math.abs(visionEstimate.getX() - currentRobotPose.getX());
        double deltaY = Math.abs(visionEstimate.getY() - currentRobotPose.getY());

        if (deltaX > 2 || deltaY > 2) {
            return false;
        }

        // Check tag area (closer tags are more reliable)
        return true;
    }

    public boolean targetPresent()
    {
        return getTag()!=null;
    }

    public double getTargetYDegrees()
    {
        if(getTag()!=null)
        {
            return getTag().getTargetYDegrees();
        }
        return 0;
    }
    public void calcDistance()
    {
        if(getTag()!=null)
        {
            distance=Math.abs(goalHeight/(Math.tan(Math.toRadians(cameraPitch+getTargetYDegrees()))));
        }
    }

    public double getDistance()
    {
        return distance;
    }

    public void getBotPose()
    {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose= result.getBotpose();
            Position poseInches= botpose.getPosition().toUnit(DistanceUnit.INCH);
            pedroPose= new Pose(
                    poseInches.x,
                    poseInches.y,
                    botpose.getOrientation().getYaw(),
                    FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
    }

    public void updateBotPose()
    {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose= result.getBotpose();
            Position poseInches= botpose.getPosition().toUnit(DistanceUnit.INCH);
            pedroPose= new Pose(
                    poseInches.y+72,
                    -poseInches.x+72,
                    botpose.getOrientation().getYaw());
        }
    }

    public Pose getPedro()
    {
        return pedroPose;
    }

    public void setMT2Orientation(Pose pose)
    {
        Pose transformed= pose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        cam.updateRobotOrientation(pose.getHeading());
    }
}
