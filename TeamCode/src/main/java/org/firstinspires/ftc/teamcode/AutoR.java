package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
                new TranslationalVelConstraint(200.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-100.0, 150.0);

        Actions.runBlocking(new SequentialAction(
                armSwing.pickup(),
                teeth.closed(),
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
                        .strafeTo(new Vector2d(43,-15))
                        .strafeTo(new Vector2d(43,-51),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(43,-15),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(53,-15),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(53,-51),baseVelConstraint,baseAccelConstraint)
                        .strafeTo(new Vector2d(53,-15),baseVelConstraint,baseAccelConstraint)
                        //.strafeTo(new Vector2d(62,-15),baseVelConstraint,baseAccelConstraint)
                        //.strafeTo(new Vector2d(62,-51),baseVelConstraint,baseAccelConstraint)
                        .build())

        ));
    }
}