package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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
        Pose2d beginPose = new Pose2d(9, -64.5, 0);
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
                wrist.num1(),
                teeth.closed()
        ));

        Actions.runBlocking(armSwing.throughBars2());
        sleep(100);
        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                // Reach out to the top bar
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(9,-38))
                        .waitSeconds(0.5)
                        .build(),
                armSwing.throughBars3()//,
                //drive.actionBuilder(new Pose2d(9,-39,0))
                //      .strafeTo(new Vector2d(9,-41))
                //    .build()

                //reach.middle()

        ));
        sleep(400);
        Actions.runBlocking(reach.inn());
        Actions.runBlocking(new SequentialAction(
                armSwing.pickup(),
                teeth.open(),
                wrist.sweep(),
                drive.actionBuilder(new Pose2d(9,-36,0))
                        .strafeTo(new Vector2d(9,-50))
                        .waitSeconds(0.2)
                        .build()
        ));
        //Go to push blocks into observation zone

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(9,-50,Math.toRadians(270)))
                        .strafeTo(new Vector2d(24,-50))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(24,-36))
                        .turnTo(315)
                        .build(),
                armSwing.neutral()
        ));
        sleep(100);
        Actions.runBlocking(new SequentialAction(
                //Block #1
                reach.out(),
                drive.actionBuilder(new Pose2d(24,-36,Math.toRadians(315)))
                        .turnTo(225)
                        .strafeTo(new Vector2d(-24,-48))
                        .build()
        ));
        sleep(10*1000);
    }
}