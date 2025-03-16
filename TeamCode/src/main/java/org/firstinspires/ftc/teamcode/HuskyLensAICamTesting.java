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
 * claw stuff:
 * 0.45 - 1
 */


// Importing things
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;


// Setup
@TeleOp(name="HuskyLensAI_Testing", group="Linear Opmode")
public class HuskyLensAICamTesting extends LinearOpMode {

    // Declare OpMode objects
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorq = null;
    private Servo servoq = null;
    private HuskyLens camq;

    //timer
    private final ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() {


        // Defining motors
        motorq = hardwareMap.get(DcMotor.class, "motorq");
        servoq = hardwareMap.get(Servo.class, "servoq");
        camq = hardwareMap.get(HuskyLens.class, "camq");

        List<HuskyLens.Block> myHuskyLensBlocks;
        HuskyLens.Block myHuskyLensBlock;
        ElapsedTime myElapsedTime;



        // Motor directions
        motorq.setDirection(DcMotor.Direction.FORWARD);

        //Declaring random CONTROL variables
        String var;

        waitForStart();
        runtime.reset();



        // Reset encoders

        motorq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Set to object classification on startup

        telemetry.addData(">>", camq.knock() ? "Touch start to continue" : "Problem communicating with HuskyLens");
        camq.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        myElapsedTime = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {

            // id 1 = yellow
            // id 2 = blue
            // id 3 = red
            // id 4+ = nothing



            // TELEMETRY

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motor", motorq.getCurrentPosition());
            telemetry.addData("Servo", servoq.getPosition());
            myHuskyLensBlocks = Arrays.asList(camq.blocks());
            telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
            for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
                myHuskyLensBlock = myHuskyLensBlock_item;
                if (myHuskyLensBlock.id == 1) {var = "yellow";}
                else if (myHuskyLensBlock.id == 2) {var = "blue";}
                else if (myHuskyLensBlock.id == 3) {var = "red";}
                else {var = null;}
                telemetry.addData("Block", "id=" + var + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y);

            }





            telemetry.update();
        }
    }
}