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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.appendeges.Wrist;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import java.util.Arrays;

@Autonomous
public class AutoYellow extends LinearOpMode {
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
      Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT Start of Auto LEFT
      ////////////////////////////////////////////////////////////////////////////////////////////*/
        telemetry.addData("Odometry x", drive.getPose().position.x);
        telemetry.addData("Odometry y", drive.getPose().position.y);
        telemetry.addData("Odo Pos x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo Pos y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));

        Actions.runBlocking(new SequentialAction(
                teeth.closed(),
                wrist.back(),
                spin.straight()
                ));

        Actions.runBlocking(armSwing.score2());
        sleep(500);
        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                // Reach out to the bucket
                reach.out(),
                drive.actionBuilder(new Pose2d(-51,-55,0))
                        .build(),

                drive.actionBuilder(new Pose2d(-54.5, -58.5, 0))
                        .turnTo(Math.toRadians(-45))
                        .build(),

                armSwing.score2()
        ));
        sleep(250);
        Actions.runBlocking(wrist.score());

        telemetry.update();

        sleep(750);
        Actions.runBlocking(teeth.open());
        sleep(125);
        Actions.runBlocking(wrist.back());
        Actions.runBlocking(armSwing.neutral());
        sleep(125);
        Actions.runBlocking(reach.inn());
        sleep(250);
        Actions.runBlocking(wrist.offset());
        //Get Second Block//////////////////////////////////////////////////////////////////////////
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-47,-39,-1))
                        .waitSeconds(0.001)
                        .turnTo(0)
                        .build(),
                spin.straight()
        ));
        Actions.runBlocking(armSwing.pickup());
        sleep(250);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        telemetry.update();
                // Reach out to the bucket
        Actions.runBlocking(new SequentialAction(
                armSwing.score2(),
                reach.out(),
                drive.actionBuilder(new Pose2d(-54, -59, 0))
                        .turnTo(Math.toRadians(-45))
                        .build(),
                armSwing.score2()
                ));
        sleep(250);
        Actions.runBlocking(wrist.score());
        sleep(750);
        Actions.runBlocking(teeth.open());
        sleep(250);
        Actions.runBlocking(wrist.back());
        telemetry.update();
        sleep(250);
        Actions.runBlocking(armSwing.wall2());
        sleep(125);
        Actions.runBlocking(reach.inn());
        Actions.runBlocking(wrist.offset());
        //Get Third Block///////////////////////////////////////////////////////////////////////////
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-57,-40,-45))
                        .turnTo(0)
                        .strafeTo(new Vector2d(-57.5,-38))
                        .build(),
                spin.straight()));
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(250);
        Actions.runBlocking(armSwing.score2());
        sleep(500);
        telemetry.update();
        // Reach out to the bucket
        Actions.runBlocking(new SequentialAction(
                reach.out(),

                drive.actionBuilder(new Pose2d(-56, -56, 0))
                        .turnTo(Math.toRadians(-45))
                        .build(),

                armSwing.score2()
        ));
        sleep(250);
        Actions.runBlocking(wrist.score());
        sleep(750);
        Actions.runBlocking(teeth.open());
        sleep(250);
        Actions.runBlocking(wrist.back());
        telemetry.update();
        sleep(250);
        Actions.runBlocking(armSwing.neutral());
        sleep(125);
        Actions.runBlocking(reach.inn());
        Actions.runBlocking(wrist.offset());
        //Get Fourth Block//////////////////////////////////////////////////////////////////////////
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-48,-29,Math.toRadians(-45)))
                        .turnTo(Math.toRadians(90))
                        .build(),
                spin.offset(),
                drive.actionBuilder(new Pose2d(-48,-29,Math.toRadians(90)))
                        .strafeTo(new Vector2d(-57.25,-29))
                        .build(),
                armSwing.pickup()));
        sleep(250);
        Actions.runBlocking(teeth.closed());
        sleep(250);
        Actions.runBlocking(armSwing.neutral());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-50,-29,Math.toRadians(90)))
                        .turnTo(0)
                        .build()
        );
        Actions.runBlocking(armSwing.score2());
        sleep(500);
        telemetry.update();
        // Reach out to the bucket
        Actions.runBlocking(new SequentialAction(
                reach.out(),

                drive.actionBuilder(new Pose2d(-53, -55, 0))
                        .turnTo(Math.toRadians(-55))
                        .build(),

                armSwing.score2()
        ));
        sleep(250);
        Actions.runBlocking(wrist.back());
        sleep(750);
        Actions.runBlocking(teeth.open());
        telemetry.update();
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                wrist.offset(),
                reach.inn(),
                armSwing.neutral()
                ));
        sleep(500);
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-50,-40,Math.toRadians(90)))
                .strafeTo(new Vector2d(-50,-20))
                        .turn(Math.toRadians(170))
                .build(),
                wrist.score(),
                reach.inn(),
                armSwing.pickup()
        ));

        sleep(10000); // So the robot doesn't destroy itself.





    }
}