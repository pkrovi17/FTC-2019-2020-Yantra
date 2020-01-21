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

package Arjun;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder V4")

public class EncoderV5 extends LinearOpMode {

    /* Declare OpMode members. */
    
    Autohardware         robot   = new Autohardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: rev Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.8;
   private Servo pullright = null;
    private Servo pullleft = null;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //sleep(1000);
        robot.init(hardwareMap);
        //sleep(1000);
        pullright = hardwareMap.get(Servo.class,"pullright");
        pullleft = hardwareMap.get(Servo.class,"pullleft");
        
        boolean myservoinit = true;
        if(myservoinit) {
                if(pullleft.getPosition() == 0.0) pullleft.setPosition(0.02);
                else pullleft.setPosition(0.02); }
        if(myservoinit) {
                if(pullright.getPosition() == 0.0) pullright.setPosition(0.1);
                else pullright.setPosition(0.1); }   

        // Send telemetry message to signify robot waiting;

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontleftd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontrightd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearleftd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearrightd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        robot.frontleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       

        // Send telemetry message to indicate successful Encoder reset
                telemetry.setAutoClear(false);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.frontleftd.getCurrentPosition(),
                          robot.frontrightd.getCurrentPosition(),
                          robot.rearleftd.getCurrentPosition(),
                          robot.rearrightd.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        
        // // Below: Go straight 71 inches - Arjun
        encoderDrive(DRIVE_SPEED,    -80, -80, 100000.0);  // S1: Forward 47 Inches with 5 Sec timeout
          
        // // Below: Change wheel diretions to turn right 90 degrees - - Arjun

        robot.frontleftd.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.frontrightd.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.rearleftd.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.rearrightd.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        encoderDrive(DRIVE_SPEED,    -17, -17.5, 10000.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // // End turning right  - Arjun
        // // Below: Change wheel directions to go straight - Arjun

        robot.frontleftd.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.frontrightd.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        robot.rearleftd.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.rearrightd.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        encoderDrive(DRIVE_SPEED,    -30, -30, 100000.0);  // S1: Forward 47 Inches with 5 Sec timeout
        
        telemetry.setAutoClear(false);
        telemetry.addData("Servo pullleft  Starting Postion", pullleft.getPosition());
        telemetry.update();
        telemetry.addData("Servo pullright  Starting Postion", pullright.getPosition());
        telemetry.update();
       //   if(pullleft.getPosition() == 0.0) pullleft.setPosition(0.0);
    //            else pullleft.setPosition(0.0); 
        
        boolean changed5 = false; //Outside of loop() 

        if(!changed5) {
                if(pullleft.getPosition() == 0.02) pullleft.setPosition(0.2);
                else pullleft.setPosition(0.2); 
                changed5 = true; }
        else changed5 = false;
          
        boolean changed6 = false; //Outside of loop() 
  
        if(!changed6) {
                if(pullright.getPosition() == 0.1) pullright.setPosition(0.01);
                else pullright.setPosition(0.01); 
                changed6 = true; }
        else changed6 = false;
        
        telemetry.setAutoClear(false);
        telemetry.addData("Servo pullleft  Ending Postion", pullleft.getPosition());
        telemetry.update();
       
       robot.frontleftd.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robot.frontrightd.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.rearleftd.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robot.rearrightd.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        encoderDrive(DRIVE_SPEED,    -30, -30, 100000.0);  // S1: Forward 47 Inches with 5 Sec timeout
        
        //  encoderDrive(TURN_SPEED,   -5, 5, 10000.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
         //encoderDrive(DRIVE_SPEED,    24, 24.0, 10000.0);  // S1: Forward 47 Inches with 5 Sec timeout
          //encoderDrive(DRIVE_SPEED,    -24, -24.0, 10000.0);  // S1: Forward 47 Inches with 5 Sec timeout


        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        // robot.rightClaw.setPosition(0.0);
        
               boolean foundback = true;
        if(foundback) {
                if(pullleft.getPosition() == 0.0) pullleft.setPosition(0.02);
                else pullleft.setPosition(0.02); }
        if(foundback) {
                if(pullright.getPosition() == 0.0) pullright.setPosition(0.1);
                else pullright.setPosition(0.1); } 
                
                
       sleep(10000);     // pause for servos to move

   //     telemetry.addData("Path", "Complete");
   //     telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfrontleftdTarget;
        int newfrontrightdTarget;
        int newrearleftdTarget;
        int newrearrightdTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontleftdTarget = robot.frontleftd.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontrightdTarget = robot.frontrightd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newrearleftdTarget = robot.rearleftd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newrearrightdTarget = robot.rearrightd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            
            robot.frontleftd.setTargetPosition(newfrontleftdTarget);
            robot.frontrightd.setTargetPosition(newfrontrightdTarget);
            robot.rearleftd.setTargetPosition(newrearleftdTarget);
            robot.rearrightd.setTargetPosition(newrearrightdTarget);
            
            

            // Turn On RUN_TO_POSITION
            robot.frontleftd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontrightd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearleftd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearrightd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            

            // reset the timeout time and start motion.
            runtime.reset();
         /* robot.frontleftd.setPower(Math.abs(speed));
            robot.frontrightd.setPower(Math.abs(speed));
            robot.rearleftd.setPower(Math.abs(speed));
            robot.rearrightd.setPower(Math.abs(speed));
        */
            robot.rearleftd.setPower(Math.abs(speed));
            robot.frontleftd.setPower(Math.abs(speed));
            sleep(225);
            robot.frontrightd.setPower(Math.abs(speed));
            robot.rearrightd.setPower(Math.abs(speed));
     
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                  (robot.frontleftd.isBusy() && robot.frontrightd.isBusy() &&
                  robot.rearleftd.isBusy() && robot.rearrightd.isBusy())) {
                // Display it for the driver.
/*                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontleftdTarget,  newrearrightdTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.frontleftd.getCurrentPosition(),
                                            robot.frontrightd.getCurrentPosition(),
                                            robot.rearleftd.getCurrentPosition(),
                                            robot.rearrightd.getCurrentPosition());
                telemetry.update();
  */
            }

            // Stop all motion;
            robot.frontleftd.setPower(0);
            robot.frontrightd.setPower(0);
            robot.rearleftd.setPower(0);
            robot.rearrightd.setPower(0);
            

            // Turn off RUN_TO_POSITION
            robot.frontleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearleftd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearrightd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
