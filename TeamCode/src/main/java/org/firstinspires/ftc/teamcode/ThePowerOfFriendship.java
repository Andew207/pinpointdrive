/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// Importing things
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="The Power Of Friendship", group="Linear Opmode")
public class ThePowerOfFriendship extends LinearOpMode {

    // Declare OpMode objects
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor armSwing = null;
    private DcMotor inOutLeft = null;
    private DcMotor inOutRight = null;
    private Servo teeth = null;
    private Servo spin = null;
    private RevTouchSensor limL;
    private RevTouchSensor limR;
    private Servo handsUpL;
    private Servo handsUpR;

    //timer
    private final ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() {


        // Find objects on Driver Controller
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rightBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        armSwing = hardwareMap.get(DcMotor.class, "armSwing");
        inOutLeft = hardwareMap.get(DcMotor.class, "inOutLeft");
        inOutRight = hardwareMap.get(DcMotor.class, "inOutRight");
        teeth = hardwareMap.get(Servo.class, "teeth");
        spin = hardwareMap.get(Servo.class, "spin");
        limL = hardwareMap.get(RevTouchSensor.class, "limL");
        limR = hardwareMap.get(RevTouchSensor.class,"limR");
        handsUpL = hardwareMap.get(Servo.class, "handsUpL");
        handsUpR = hardwareMap.get(Servo.class, "handsUpR");


        int slow = 1;


        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        inOutLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        inOutRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armSwing.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        runtime.reset();

        spin.setPosition(0.5);

        // Reset encoders

        inOutLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inOutRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inOutLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inOutRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run with encoder


        // Declare random variables
        boolean changed = false;
        boolean changed1 = false;
        boolean changed2 = false;

        double drive;
        double strafe;
        double turn;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double armPower;
        int inOutPosition = 0;
        double teethPos = 0;
        int armSwingPosition = 0;
        double bumper = 0;
        double hands = 0.2;



        while (opModeIsActive()) {


            // Drive variables
            drive = -gamepad1.left_stick_x;
            strafe = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Setting the 3 intake servos

            // slow mode! //
            if (gamepad1.y && !changed) {
                if (slow == 1) slow = 2;
                else slow = 1;
                changed = true;
            } else if (!gamepad1.y) changed = false;

            if (gamepad2.left_trigger - gamepad2.right_trigger != 0){
                if (gamepad2.left_trigger != 0){
                    inOutPosition = inOutPosition + 80;
                }
                else{
                    inOutPosition = inOutPosition - 80;
                }
            }
            if (inOutPosition < 0 && limL.isPressed() && limR.isPressed()){
                inOutLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                inOutRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                inOutPosition = 40;}
            if (inOutPosition >= 2550)
                inOutPosition = 2500;


            if (gamepad2.x && !changed1) {
                if (spin.getPosition() == 1) spin.setPosition(0);
                else spin.setPosition(1);
                changed1 = true;
            } else if (!gamepad2.x) changed1 = false;

            if (gamepad2.a && !changed2) {
                if (teethPos == 1) teethPos = 0.337;
                else teethPos = 1;
                changed2 = true;
            } else if(!gamepad2.a) changed2 = false;

            //bumper is a control variable
            if(gamepad1.left_bumper)
                bumper = 1;
            else if(gamepad1.right_bumper)
                bumper = -1;
            else
                bumper = 0;

            if(bumper != 0){
                if (bumper > 0){
                    armSwingPosition += 20;
                }
                else
                    armSwingPosition -= 20;
                if (armSwingPosition >= 20){
                    armSwingPosition = -20;
                }
            }
            if (armSwingPosition < -1900){
                armSwingPosition = -1890;
            }

            if(gamepad2.b) {

                    armSwingPosition = -1990;
                    inOutPosition = 3500;

            }

            if (gamepad2.right_bumper)hands = 0.7;
            if (gamepad2.left_bumper)hands = 0.5;
            if (gamepad2.a)hands = 0.2;


            if(armSwingPosition >= -400){
                armPower = 0.5;
            }
            else{
                armPower = 1;
            }

            armSwing.setTargetPosition(armSwingPosition);

            inOutRight.setTargetPosition(inOutPosition);
            inOutLeft.setTargetPosition(inOutPosition);

            armSwing.setPower(armPower);

            inOutRight.setPower(1);
            inOutLeft.setPower(1);

            armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            inOutRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            inOutLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);






            handsUpL.setPosition(hands);
            handsUpR.setPosition(hands);





            teeth.setPosition(teethPos);



            // Drive equations
            frontLeftPower = Range.clip((drive + strafe - turn) / slow, -1, 1);
            frontRightPower = Range.clip((drive - strafe - turn) / slow, -1, 1);
            backLeftPower = Range.clip((drive - strafe + turn) / slow, -1, 1);
            backRightPower = Range.clip((drive + strafe + turn) / slow, -1, 1);

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);


            // TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());

            telemetry.addData("Lim L", limL.isPressed());
            telemetry.addData("Lim R", limR.isPressed());

            telemetry.addData("teeth", teeth.getPosition());
            telemetry.addData("spin", spin.getPosition());

            telemetry.addData("target pos var", inOutPosition);
            telemetry.addData("left pos", inOutLeft.getCurrentPosition());
            telemetry.addData("right pos", inOutRight.getCurrentPosition());
            telemetry.addData("Arm Swing", armSwing.getCurrentPosition());

            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);


            telemetry.update();
        }
    }

}