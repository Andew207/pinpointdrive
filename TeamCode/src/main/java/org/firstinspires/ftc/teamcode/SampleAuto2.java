package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
//import org.firstinspires.ftc.teamcode.drive.SpecimenClaw;
//import org.firstinspires.ftc.teamcode.drive.ScoringArm;

@Autonomous
public class SampleAuto2 extends OpMode {
 //   SpecimenClaw scoringClaw;
 //   ScoringArm scoringArm;
    Pose2d beginPose;
    PinpointDrive drive;

    public void init() {
 //       scoringClaw = new SpecimenClaw(hardwareMap);
 //       scoringArm = new ScoringArm(hardwareMap);
        beginPose = new Pose2d(0, 0, -Math.PI / 2);
        drive = new PinpointDrive(hardwareMap, beginPose);
    }

    public void start() {
        //Actions.runBlocking(scoringClaw.close());
        //Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-60, 0))
                        .build()
        );
       //Actions.runBlocking(scoringClaw.open());
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.PI/2)
                      //  .splineToConstantHeading(new Vector2d(-27.5, 80), Math.PI)
                      //  .splineToConstantHeading(new Vector2d(-35, 66), Math.PI)
                        .turn(Math.PI)
                        .build()
        );
        //Actions.runBlocking(scoringClaw.close());
    }

    public void loop() {}
}
