package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;

import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import javax.crypto.ExemptionMechanism;

@Autonomous
public class Auto extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-8, -64, 0);
        Pose2d pose = new Pose2d(0,0,0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        Actions.runBlocking(teeth.Closed());

        waitForStart();

        Actions.runBlocking(armSwing.throughBars1());

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeTo(new Vector2d(-8,-40))

                        .build()

        );
        Actions.runBlocking(armSwing.throughBars2());
        sleep(500);
        Actions.runBlocking(teeth.Open());
        sleep(1000);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-8,-40, 0))
                        .strafeTo(new Vector2d(-8,-50))
                        .build()
        );


    }
}