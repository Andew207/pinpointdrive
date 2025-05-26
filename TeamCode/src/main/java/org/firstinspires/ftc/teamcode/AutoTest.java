package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.appendeges.Wrist;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import java.util.Arrays;

@Autonomous
public class AutoTest extends LinearOpMode {
    /*
     * Left Side Auto
     */
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-38.5, -64.5, 0);
        Pose2d pose = new Pose2d(0,0,0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);




        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);


        armSwing.init();



        waitForStart();


        // Speed constraints //
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 50.0);


      /*////////////////////////////////////////////////////////////////////////////////////////////
      Start of Auto TEST Start of Auto TEST Start of Auto TEST Start of Auto TEST Start of Auto TEST
      ////////////////////////////////////////////////////////////////////////////////////////////*/
        telemetry.addData("Odometry x", drive.getPose().position.x);
        telemetry.addData("Odometry y", drive.getPose().position.y);
        telemetry.addData("Odo Pos x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo Pos y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-50,-50,0))
                        .turnTo(Math.toRadians(45))
                        .turnTo(Math.toRadians(90))
                        .turnTo(Math.toRadians(135))
                        .turnTo(Math.toRadians(180))
                        .turnTo(Math.toRadians(225))
                        .turnTo(Math.toRadians(270))
                        .turnTo(Math.toRadians(315))
                        .turnTo(Math.toRadians(0))
                        .build()
        ));
    }
}