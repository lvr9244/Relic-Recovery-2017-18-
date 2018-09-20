/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This autonomous's goal is to move forward, knock the cap ball off of the center, and park in
 * the center vortex. It is designed to be used with four sets of omni wheels.
 *
 * I also was having a little too much fun with the telemetry. :)
 */

@Autonomous(name="Auto Blue With Jewels", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto_Blue_W_Jewels extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorRear = null;
    private DcMotor leftMotorRear = null;

    private Servo jewelArm;

    ColorSensor colorSensor;
    DigitalChannel touchSensor;

    long jewelKnockTime = 800;
    double redValue = 400;
    double pushSpeed = .2;

    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotorFront  = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorRear = hardwareMap.dcMotor.get("rightMotorRear");
        leftMotorRear = hardwareMap.dcMotor.get("leftMotorRear");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelArm.scaleRange(0, 90);
        // get a reference to our digitalTouch object.
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        // set the digital channel to input.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        //touchWall = hardwareMap.touchSensor.get("touchWall");

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        runtime.reset();
        runtime.startTime();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // hey jenna youre pretty decent ;)
        //that was anwyn
        //this is steve
        //youre not the worst



        jewelArm.setPosition(.9);
        sleep(300);

        //encoderDrive(1, 12, 12, 12, 12, 5);


/*
        //the robot will drive forward until the touch sensor is pressed
        if (touchSensor.getState() == false){
            telemetry.addData("touch" , "pressed");
            motorOff();
        } else {
            driveBackward(.2);
        }
*/

        driveForward(.2);
        sleep(300);
        motorOff();

        sleep(500);

        if (colorSensor.red() > colorSensor.blue()){
            telemetry.addData("Ball", "Red");
            telemetry.update();
            sleep(1000);

            driveLeft(pushSpeed);
            sleep(jewelKnockTime);
            motorOff();

        } else if (colorSensor.red() < colorSensor.blue()){

            telemetry.addData("Ball", "Blue");
            telemetry.update();

            sleep(1000);

            driveRight(pushSpeed);
            sleep(jewelKnockTime);
            motorOff();
        }

        sleep(100);
        jewelArm.setPosition(.3);

        motorOff();
        // heyyyyy juuuuuuules

    }
    public void motorOff(){

        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorRear.setPower(0);
        rightMotorRear.setPower(0);
    }
    public void driveForward (double speed) {
        leftMotorFront.setPower(speed);
        rightMotorFront.setPower(speed);
        leftMotorRear.setPower(speed);
        rightMotorRear.setPower(speed);
    }
    public void driveLeft (double speed) {
        leftMotorFront.setPower(-speed);
        rightMotorFront.setPower(speed);
        leftMotorRear.setPower(speed);
        rightMotorRear.setPower(-speed);
    }
    public void driveRight (double speed) {
        leftMotorFront.setPower(speed);
        rightMotorFront.setPower(-speed);
        leftMotorRear.setPower(-speed);
        rightMotorRear.setPower(speed);
    }
    public void driveBackward (double speed) {
        leftMotorFront.setPower(-speed);
        rightMotorFront.setPower(-speed);
        leftMotorRear.setPower(-speed);
        rightMotorRear.setPower(-speed);
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftRearInches, double rightRearInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftMotorFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftRearTarget = leftMotorRear.getCurrentPosition() + (int)(leftRearInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightMotorFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightRearTarget = rightMotorRear.getCurrentPosition() + (int)(rightRearInches * COUNTS_PER_INCH);
            leftMotorFront.setTargetPosition(newLeftFrontTarget);
            rightMotorFront.setTargetPosition(newRightFrontTarget);
            leftMotorRear.setTargetPosition(newLeftRearTarget);
            rightMotorRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));
            leftMotorRear.setPower(Math.abs(speed));
            rightMotorRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotorFront.isBusy() && rightMotorFront.isBusy() && leftMotorRear.isBusy() && rightMotorRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget,  newRightRearTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotorFront.getCurrentPosition(),
                        rightMotorFront.getCurrentPosition(),
                        leftMotorRear.getCurrentPosition(),
                        rightMotorRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotorFront.setPower(0);
            rightMotorFront.setPower(0);
            leftMotorRear.setPower(0);
            rightMotorRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    }


