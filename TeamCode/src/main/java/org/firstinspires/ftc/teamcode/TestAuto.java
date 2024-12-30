package org.firstinspires.ftc.teamcode.autonomous.park;

import com.acmerobotics.roadrunner.CompositeAccelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PinpointDrive;

@Autonomous
public class TestAuto extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // SAMPLE STRAFE LEFT OR RIGHT
                        //  .strafeTo(new Vector2d(-10, 0)) // if starting at 0/0 this will move 10" in negative direction
                        //  .strafeTo(new Vector2d(0, 0))

                        // SAMPLE TURNS
                        //  .turnTo(0)              // Turn to heading, if you start at heading zero should do nothing.
                        //.turnTo(Math.PI/2)        // Turn to heading +90 degrees

                        // SAMPLE SPLINE to NEW HEADING
                        //.splineToLinearHeading(
                        //        new Pose2d(
                        //                new Vector2d(10, 10),
                        //                Rotation2d.fromDouble(Math.PI/2)
                        //        ), Math.PI
                        // )

                        // SAMPLE DRIVE STRAIGHT with use of velocity and acceleration functions.  Notice behavior of robot
                        .lineToX(-40)                                                 // Drive to position 0,-40, max velocity and accel
                        .lineToX(0, new TranslationalVelConstraint(10.0))   // Return to start slow velocity
                        .lineToX(-40, null, new ProfileAccelConstraint(-10.0, 40.0))   // Drive forward to position 0,-40 acceleration quickly
                        .lineToX(0, new TranslationalVelConstraint(5.0), new ProfileAccelConstraint(-10.0, 10.0))   // return to 0,0 accelerate slowly at low low velocity


                        // SAMPLE DRIVE IN RECTANGLE WITH DIFFERENT TURNS

/*
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(0,-66))
                        .strafeTo(new Vector2d(-20, -66))
                        //.lineToY(-12)
                        //.turnTo(0)
                        .splineTo( new Vector2d(0,-44), 0)
                        .strafeTo(new Vector2d(-0, 0))
                        .turnTo(0)

 */
                        .build()

        );
/*        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0,0))
                        .build()
        ); */
    }
}