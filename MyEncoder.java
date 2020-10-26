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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Team Yantra's Encoder Code for Computer Vision

@Autonomous(name="MyEncoder")

public class MyEncoder extends LinearOpMode {
    //Declare Motors
    DcMotor frontleftd = null;
    DcMotor frontrightd = null;
    DcMotor rearleftd = null;
    DcMotor rearrightd = null;
    int newfrontleftdTarget;
    int newfrontrightdTarget;
    int newrearleftdTarget;
    int newrearrightdTarget;
    double leftInches;
    double rightInches;


    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Motors
        frontleftd = hardwareMap.get(DcMotor.class, "frontleftd");
        frontrightd = hardwareMap.get(DcMotor.class, "frontrightd");
        rearleftd = hardwareMap.get(DcMotor.class, "rearleftd");
        rearrightd = hardwareMap.get(DcMotor.class, "rearrightd");

        frontleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: rev Motor Encoder
    final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

        public void forward_power(double power){
            frontleftd.setPower(power);
            frontrightd.setPower(power);
            rearleftd.setPower(power);
            rearrightd.setPower(power);
        }
        public void left_power(double power){
        frontleftd.setPower(power);
        frontrightd.setPower(-power);
        rearleftd.setPower(power);
        rearrightd.setPower(-power);

    }

    public void right_power(double power){
        frontleftd.setPower(-power);
        frontrightd.setPower(power);
        rearleftd.setPower(-power);
        rearrightd.setPower(-power);

    }
    public void zeropower(){
            frontleftd.setPower(0);
            frontrightd.setPower(0);
            rearleftd.setPower(0);
            rearrightd.setPower(0);
    }
    public void drive_forward(double power,){
        //Reset encoders
        frontleftd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearleftd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearrightd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set targets

        newfrontleftdTarget = frontleftd.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newfrontrightdTarget = frontrightd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newrearleftdTarget = rearleftd.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newrearrightdTarget = rearrightd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        frontleftd.setTargetPosition(newfrontleftdTarget);
        frontrightd.setTargetPosition(newfrontrightdTarget);
        rearleftd.setTargetPosition(newrearleftdTarget);
        rearrightd.setTargetPosition(newrearrightdTarget);

        // Turn on run to position

        frontleftd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearleftd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearrightd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set speed
        MyEncoder speeds = new MyEncoder();
        speeds.forward_power(0.5);

        // Loop to let motors run until target reached
        while(frontleftd.isBusy() && frontrightd.isBusy() && rearleftd.isBusy() && rearrightd.isBusy()){
            telemetry.addData("Running Encoders"," ");
        }
        speeds.zeropower();

        frontleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);









    }
}
