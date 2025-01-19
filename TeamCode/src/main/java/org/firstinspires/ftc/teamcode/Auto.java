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

import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;

import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import java.util.Arrays;

import javax.crypto.ExemptionMechanism;

@Autonomous
public class Auto extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-32, -64, 0);
        Pose2d pose = new Pose2d(0,0,0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        Actions.runBlocking(teeth.open());
        Actions.runBlocking(spin.straight());



        waitForStart();

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 50.0);

        Actions.runBlocking(new SequentialAction(
                armSwing.neutral(),
                spin.straight(),
                new ParallelAction(
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(-47,-32.5), baseVelConstraint, baseAccelConstraint)
                                .build()
                )
        ));




        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        sleep(500);

        /*
        Actions.runBlocking(armSwing.pickup(),
                sleep(500),
                teeth.closed(),
                sleep(500),
                armSwing.score1(),
                sleep(500),
                reach.out());
        */
        Actions.runBlocking(new SequentialAction(
                reach.out(),

                drive.actionBuilder(new Pose2d(-52, -52, 0))
                        .turn(Math.toRadians(135))
                        .build(),

                armSwing.score2()
        ));




        sleep(  500);
        Actions.runBlocking(teeth.open());
        sleep(500);
        Actions.runBlocking(armSwing.score1());

        sleep(500);
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                drive.actionBuilder(new Pose2d(-52,-52, 3*Math.PI/4))
                        .strafeTo(new Vector2d(-47,-47), baseVelConstraint, baseAccelConstraint)
                        .build(),
                reach.inn()),
                new ParallelAction(
                drive.actionBuilder(new Pose2d(-47,-47, 3*Math.PI/4))
                        .turn(Math.toRadians(-135))
                        .build(),
                armSwing.neutral())
        ));



        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47,-47,0))
                        .lineToX(-57)
                        .strafeTo(new Vector2d(-57,-32.5))
                        .build()
        );
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        Actions.runBlocking(armSwing.score1());
        sleep(500);
        Actions.runBlocking(reach.out());

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47, -32.5, 0))
                        .strafeTo(new Vector2d(-47,-45))
                        .turn(Math.toRadians(135))
                        .build()
        );


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47,-45, 3*Math.PI/4))
                        .strafeTo(new Vector2d(-52,-52))
                        .build()
        );
        Actions.runBlocking(armSwing.score2());
        sleep(1000);
        Actions.runBlocking(teeth.open());

        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-52,-52, 3*Math.PI/4))
                        .strafeTo(new Vector2d(-47,-47))
                        .build()
        );
        Actions.runBlocking(reach.inn());

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47,-47, 3*Math.PI/4))
                        .turn(Math.toRadians(-90))
                        .build()
        );
        Actions.runBlocking(spin.offset());
        Actions.runBlocking(armSwing.corner());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-46,-40,Math.PI/4))
                        .strafeTo(new Vector2d(-55,-35))
                        .build()
        );
        Actions.runBlocking(reach.middle());
        sleep(750);
        Actions.runBlocking(armSwing.pickup());
        sleep(500);
        Actions.runBlocking(teeth.closed());
        sleep(500);
        Actions.runBlocking(reach.inn());
        Actions.runBlocking(armSwing.score1());
        sleep(500);
        Actions.runBlocking(reach.out());

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47, -32.5, 0))
                        .strafeTo(new Vector2d(-47,-45))
                        .turn(Math.toRadians(135))
                        .build()
        );


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-47,-45, 3*Math.PI/4))
                        .strafeTo(new Vector2d(-52,-52))
                        .build()
        );
        Actions.runBlocking(armSwing.score2());
        sleep(1000);
        Actions.runBlocking(teeth.open());


    }
}