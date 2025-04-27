package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

import java.util.Arrays;

@Autonomous
public class AutoR extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(15, -64, 0);
        Pose2d pose = new Pose2d(0,0,0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);





        waitForStart();



        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(140),
                new AngularVelConstraint(Math.PI / 3)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-50.0, 100.0);

        VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(25),
                new AngularVelConstraint(Math.PI / 4)
        ));
        AccelConstraint slowAccelConstraint = new ProfileAccelConstraint(-20.0, 20.0);


        Actions.runBlocking(new SequentialAction(

                teeth.closed(),
                armSwing.pickup(),
                armSwing.throughBars1(),
                new ParallelAction(
                        spin.straight(),
                        drive.actionBuilder(beginPose)
                                .strafeTo(new Vector2d(8,-39))
                                .build()
                ),
                armSwing.throughBars2(),
                drive.actionBuilder(new Pose2d(8,-39, 0))
                        .strafeTo(new Vector2d(8,-45))
                        .build(),
                teeth.open(),
                new SequentialAction(
                armSwing.neutral(),
                drive.actionBuilder(new Pose2d(8,-45, 0))
                        .splineTo(new Vector2d(36,-15), Math.toRadians(90), new AngularVelConstraint(Math.PI/4))
                        .strafeTo(new Vector2d(43,-15),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(43,-45),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(43,-18),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(54.5,-17),baseVelConstraint,baseAccelConstraint)
                        .build())
        ));

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(56,-20, Math.PI/2))

                        .strafeTo(new Vector2d(56,-46),baseVelConstraint,baseAccelConstraint)

                .build(),
                armSwing.wall()));

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(53,-51, Math.PI))
                        .strafeTo(new Vector2d(47.5,-55), slowVelConstraint, slowAccelConstraint)

                        .strafeTo(new Vector2d(47.5,-59), slowVelConstraint, slowAccelConstraint)
                        .build(),
                teeth.closed()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction( new ParallelAction(
                armSwing.throughBars1(),
                drive.actionBuilder(new Pose2d(47.5, -60, Math.PI))
                        .strafeTo(new Vector2d(47.5, -50))
                        .build()
        )));


        Actions.runBlocking(new SequentialAction(
                
                new ParallelAction(
                        spin.straight(),
                        drive.actionBuilder(new Pose2d(47.5, -50, 0))
                                .strafeTo(new Vector2d(6,-45))
                                .strafeTo(new Vector2d(6,-39))
                                .build()
                ),
                new SleepAction(0.1),
                armSwing.throughBars3(),
                drive.actionBuilder(new Pose2d(6,-39, 0))
                        .strafeTo(new Vector2d(6,-40))
                        .build(),
                new SleepAction(0.1),
                teeth.open(),
                new SleepAction(0.1)));
        Actions.runBlocking(new SequentialAction(
                armSwing.wall(),
                drive.actionBuilder(new Pose2d(6,-42, Math.PI))
                        .strafeTo(new Vector2d(47.5,-55))
                        .strafeTo(new Vector2d(47.5,-55), slowVelConstraint, slowAccelConstraint)

                        .strafeTo(new Vector2d(47.5,-59), slowVelConstraint, slowAccelConstraint)
                        .build(),
                teeth.closed()
        ));
        sleep(500);
        Actions.runBlocking(new SequentialAction( new ParallelAction(
                armSwing.throughBars2(),
                drive.actionBuilder(new Pose2d(47.5, -60, Math.PI))
                        .strafeTo(new Vector2d(47.5, -50))
                        .build()
        )));


        Actions.runBlocking(new SequentialAction(

                new ParallelAction(
                        spin.straight(),
                        drive.actionBuilder(new Pose2d(47.5, -50, 0))
                                .strafeTo(new Vector2d(2,-45))
                                .strafeTo(new Vector2d(2,-39))
                                .build()
                ),
                new SleepAction(0.1),
                armSwing.throughBars3(),
                drive.actionBuilder(new Pose2d(2,-39, 0))
                        .strafeTo(new Vector2d(2,-40))
                        .build(),
                new SleepAction(0.1),
                teeth.open(),
                new SleepAction(0.1),
                armSwing.neutral(),
                drive.actionBuilder(new Pose2d(2, -42,0))
                        .strafeTo(new Vector2d(64,-66), baseVelConstraint, baseAccelConstraint)
                        .build()
        ));
    }
}