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
public class AutoPurple extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(9, -64.5, 0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);




        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);


        Actions.runBlocking(armSwing.init());



        waitForStart();


        // Speed constraints //
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 50.0);


 /*/////////////////////////////////////////////////////////////////////////////////////////////////
 Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT
 /////////////////////////////////////////////////////////////////////////////////////////////////*/
        telemetry.addData("Odometry x", drive.getPose().position.x);
        telemetry.addData("Odometry y", drive.getPose().position.y);
        telemetry.addData("Odo Pos x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo Pos y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));

        Actions.runBlocking(new SequentialAction(
                teeth.closed(),
                wrist.back()
                ));

        Actions.runBlocking(armSwing.throughBars1());
        sleep(500);
        telemetry.update();


        Actions.runBlocking(
                // Reach out to the top bar
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(9,-36))
                        .build()
        );
        sleep(250);
        Actions.runBlocking(teeth.open());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(9,-36,0))
                        .strafeTo(new Vector2d(9,-50))
                        .waitSeconds(0.25)
                        .build()
        );
        //Go to push blocks into observation zone
        sleep(250);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(36,-50,0))
                        .waitSeconds(0.75)
                        .strafeTo(new Vector2d(36,-12))
                        .build()
        );
        sleep(250);
        Actions.runBlocking(
                //Block #1
                drive.actionBuilder(new Pose2d(48,-12,0))
                        .strafeTo(new Vector2d(48,-60))
                        .build()
        );
        sleep(250);
        Actions.runBlocking(
                //Block #2
                drive.actionBuilder(new Pose2d(48,-12,0))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(56,-12))
                        .build()
        );
        sleep(250);
        Actions.runBlocking(
                //block #3
                drive.actionBuilder(new Pose2d(56,-12,0))
                        .strafeTo(new Vector2d(56,-60))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(56,-12))
                        .build()
        );
        sleep(250);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(60,-12,0))
                        .strafeTo(new Vector2d(60,-60))
                        .waitSeconds(0.1)
                        //go out so that the person can grab blocks
                        .strafeTo(new Vector2d(48,-40))
                        .waitSeconds(0.1)
                        .build()
        );
        //grab clipped blocks
        Actions.runBlocking(new SequentialAction(
                //block #1 to bar
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .build(),
                teeth.open(),
                armSwing.pickup()
        ));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(0,-36))
                        .build(),
                armSwing.throughBars1()
        ));
        Actions.runBlocking(new SequentialAction(
                //block #2 to bar
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .build(),
                teeth.open(),
                armSwing.pickup()
        ));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(2,-36))
                        .build(),
                armSwing.throughBars1()
        ));
        Actions.runBlocking(new SequentialAction(
                //block #3 to bar
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .build(),
                teeth.open(),
                armSwing.pickup()
        ));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(4,-36))
                        .build(),
                armSwing.throughBars1()
        ));
        Actions.runBlocking(new SequentialAction(
                //park robot, ending setup
                drive.actionBuilder(new Pose2d(4,-40,0))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(-60,-64))
                        .build(),
                wrist.score(),
                armSwing.pickup(),
                reach.inn()
        ));


        sleep(10000); // So the robot doesn't destroy itself.





    }
}