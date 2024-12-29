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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Kypten's Dumb Layout The First", group="Linear Opmode")
public class KyptensDumbLayoutTheFirst extends LinearOpMode {

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

        int slow = 1;


        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        inOutLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        runtime.reset();

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inOutLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inOutRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inOutLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inOutRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run with encoder
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Declare random variables
        boolean changed = false;
        boolean changed1 = false;

        double drive;
        double strafe;
        double turn;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        int inOutPosition = 0;
        double teethPos = 0;
        int armSwingPosition = 0;



        while (opModeIsActive()) {


            // Drive variables
            drive = -gamepad1.left_stick_x;
            strafe = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Setting the 3 intake servos

            // slow mode! //
            if (gamepad1.a && !changed) {
                if (slow == 1) slow = 2;
                else slow = 1;
                changed = true;
            } else if (!gamepad1.a) changed = false;

            if (gamepad2.left_trigger - gamepad2.right_trigger != 0){
                if (gamepad2.left_trigger != 0){
                    inOutPosition = inOutPosition + 40;
                }
                else{
                    inOutPosition = inOutPosition - 40;
                }
            }
            if (inOutPosition <= -20)
                inOutPosition = 40;
            if (inOutPosition >= 3100)
                inOutPosition = 3000;




            if (gamepad2.a && !changed1) {
                if (teethPos == 1) teethPos = 0;
                else teethPos = 1;
                changed1 = true;
            } else if (!gamepad2.a) changed1 = false;

            if(gamepad2.left_stick_y != 0){
                if (gamepad2.left_stick_y > 0){
                    armSwingPosition += 20;
                }
                else
                    armSwingPosition -= 20;
                if (armSwingPosition >= 20){
                    armSwingPosition = -20;
                }
            }

            armSwing.setTargetPosition(armSwingPosition);

            inOutRight.setTargetPosition(inOutPosition);
            inOutLeft.setTargetPosition(inOutPosition);

            armSwing.setPower(1);

            inOutRight.setPower(1);
            inOutLeft.setPower(1);

            //armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            inOutRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            inOutLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);











            teeth.setPosition(teethPos);



            // Drive equations
            frontLeftPower = Range.clip((drive + strafe - turn) / slow, -0.75, 0.75);
            frontRightPower = Range.clip((drive - strafe - turn) / slow, -0.75, 0.75);
            backLeftPower = Range.clip((drive - strafe + turn) / slow, -0.75, 0.75);
            backRightPower = Range.clip((drive + strafe + turn) / slow, -0.75, 0.75);

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