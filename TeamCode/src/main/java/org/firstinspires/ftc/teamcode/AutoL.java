package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;

import org.firstinspires.ftc.teamcode.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import java.util.Arrays;

@Autonomous
public class AutoL extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-8, -64, 0);
        Pose2d pose = new Pose2d(0,0,0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);




        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);





        waitForStart();


        // Speed constraints //
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 50.0);


      /*////////////////////////////////////////////////////////////////////////////////////////////
      Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT
      ////////////////////////////////////////////////////////////////////////////////////////////*/
        Actions.runBlocking(new SequentialAction(
                // Put first SPECIMEN on bars
                armSwing.pickup(),
                teeth.closed(),
                armSwing.throughBars1(),
                new ParallelAction(
                        spin.straight(),
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(-8,-39))

                                .build()
                    ),
                armSwing.throughBars2(),
                // Go to the left side to grab first block
                drive.actionBuilder(new Pose2d(-8,-39, 0))
                        .strafeTo(new Vector2d(-8,-45))
                        .build(),
                teeth.open(),
                armSwing.neutral(),
                drive.actionBuilder(new Pose2d(-8,-45, 0))
                        .strafeTo(new Vector2d(-28,-45))
                        .build()
        ));

        telemetry.addData("Odometry x", drive.getPose().position.x);
        telemetry.addData("Odometry y", drive.getPose().position.y);
        telemetry.addData("Odo Pos x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo Pos y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));



        telemetry.update();
        Actions.runBlocking(new SequentialAction(


                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-28, -45, 0))
                                .strafeTo(new Vector2d(-47,-36), baseVelConstraint, baseAccelConstraint)
                                .build()
                )
        ));


        telemetry.update();
        // Actually grab the first block
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        telemetry.update();
        Actions.runBlocking(teeth.closed());
        sleep(500);
        telemetry.update();
        Actions.runBlocking(armSwing.score1());
        sleep(500);
        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                // Reach out to the bucket
                reach.out(),

                drive.actionBuilder(new Pose2d(-52, -52, 0))
                        .turn(Math.toRadians(135))
                        .build(),

                armSwing.score2()
        ));



        // Let go of the first block
        sleep(1000);
        Actions.runBlocking(teeth.open());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        sleep(250);

        //Pull back & go to 2nd block
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                drive.actionBuilder(new Pose2d(-52,-52, 0))
                        .strafeTo(new Vector2d(-47,-47), baseVelConstraint, baseAccelConstraint)
                        .build(),

                reach.inn()),
                armSwing.neutral()
        ));




        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47,-47,0))
                        .strafeTo(new Vector2d(-57,-36))
                        .build()
        );
        // Grab 2nd block
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        sleep(500);

        //Score the 2nd block
        Actions.runBlocking(new SequentialAction(
                reach.out(),

                drive.actionBuilder(new Pose2d(-52, -52, 0))
                        .turn(Math.toRadians(135))
                        .build(),

                armSwing.score2()
        ));




        sleep(500);
        Actions.runBlocking(teeth.open());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        //Go to 3rd block
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-52,-52, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-48,-27.5), baseVelConstraint, baseAccelConstraint)
                                .build(),

                        reach.inn())
        ));
        Actions.runBlocking(armSwing.corner());
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                spin.offset(),
                drive.actionBuilder(new Pose2d(-48,-27.5,Math.toRadians(90)))
                        .strafeTo(new Vector2d(-58,-27.5))
                        .build()
        ));
        //Grab 3rd block
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                armSwing.neutral(),
                spin.straight(),
                teeth.closed(),
                drive.actionBuilder(new Pose2d(-58,-27.5,Math.toRadians(90)))
                        .strafeTo(new Vector2d(-48,-27.5))
                        .build(),
                armSwing.score1()
        ));



        //Score 3rd block
        Actions.runBlocking(new SequentialAction(
                reach.out(),

                drive.actionBuilder(new Pose2d(-52, -52, 0))
                        .turn(Math.toRadians(135))
                        .build(),

                armSwing.score2()
        ));



        //Score 3rd block
        sleep(500);
        Actions.runBlocking(teeth.open());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        sleep(250);
        //End auto, and reset robot for later Teleop usage.
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-52,-52, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-48,-23), baseVelConstraint, baseAccelConstraint)
                                .build(),

                        reach.inn())
        ));
        Actions.runBlocking(armSwing.corner());




    }
}