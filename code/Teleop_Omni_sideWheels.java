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
package org.firstinspires.ftc.teamcode; //You'll never be Josh -- Yes but im better

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 *
 *
 *
 * FIGURE OUT ENCODERS FOR THE TURNING ARM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *
 *
 *
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp--Omni exp side wheel", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Teleop_Omni_sideWheels extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontMotor = null;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor= null;
    private DcMotor backMotor = null;
    //private DcMotor liftMotor = null;
    //private DcMotor turnMotor = null;

     Servo leftArm;
     Servo rightArm;

    boolean turnRight;
    boolean turnLeft; // Do you even code? -- no
    boolean right;
    boolean left; // boo lean? i thought better of you than drugs, jenna -- college isn't going to pay for itself
    boolean cutSpeed;
    boolean leftArmOut;

    double continuousStop = .15 ;
    double horizontal; // variable for horizontal movement
    double vertical; // variable for vertical movement
    double rotational; // variable for rotational movement
    double deadZone = .1;
        int j; // variable for lift for loop




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Jenna:", "Don't forget to press play. It won't work if you don't press play");


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        frontMotor  = hardwareMap.dcMotor.get("frontMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("" + "rightMotor");
        backMotor = hardwareMap.dcMotor.get("backMotor");
        //turnMotor = hardwareMap.dcMotor.get("turnMotor");
        //liftMotor = hardwareMap.dcMotor.get("liftMotor");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        frontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
       // turnMotor.setDirection(DcMotor.Direction.FORWARD);
        //liftMotor.setDirection(DcMotor.Direction.FORWARD);

       // buttonpusher.setPosition(continuousStop);
        leftArmOut = false; // leftArm is in home position

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("GO LENAPE", "WAHOOOO");
        //telemetry.addData("Jenna tie your shoe","~Ludovica");
        //telemetry.addData("Preterintenzionale", ""); // I was having way too much fun with telemetry


        /*
            * The idea is that the different variables represent a direction for the omni wheels.
            * For example, when the robot moves forward, the motors all move forward (the right
            * side is reversed). Therefore, the vertical variable is positive for all of the motors.
            * To go sideways, the front right and back left must move forward, while the other
            * two motors must move backwards, which is why the horizontal variable is negative for
            * two of the motors.

        */

        vertical = -gamepad1.left_stick_y; // vertical movement is controlled by up and down on the right stick
        horizontal = -gamepad1.left_stick_x; // horizontal movement is controlled by side   to side on the right stick
        rotational = gamepad1.right_stick_x; // pirouetting is controlled by side to side on the left stick

        if ( !leftArmOut){     // this is all that is needed to move the left arm out and keep it there!
            leftArm.setPosition(-1);
            leftArmOut = true;  // by making leftArmOut 'True' this routine will never be hit again.
        }


        if (gamepad1.left_bumper && !cutSpeed){
            cutSpeed = true;
        } else if (gamepad1.right_bumper && cutSpeed){
            cutSpeed = false;
        }

        if (cutSpeed) {
            frontMotor.setPower((-horizontal + rotational) / 5); //Jenna is pretty dumb and doesn't know how to program -- very true
            backMotor.setPower((horizontal + rotational) / 5);
            leftMotor.setPower((vertical - rotational) / 5);
            rightMotor.setPower((vertical + rotational) / 5);

        } else if (!cutSpeed) {
            frontMotor.setPower(-horizontal + rotational); //Jenna is pretty dumb and doesn't know how to program -- very true
            backMotor.setPower(horizontal + rotational);
            leftMotor.setPower(vertical - rotational);
            rightMotor.setPower(vertical + rotational);

        }
//arm is programmed to x and b
        if (gamepad2.x){
            rightArm.setPosition(1);
            leftArm.setPosition(1);

        } else if (gamepad2.b){
            rightArm.setPosition(0);
            leftArm.setPosition(0);

        }

        if (gamepad2.left_trigger > deadZone) {              // Doing a nested if else allows for smooth operation
            //liftMotor.setPower(gamepad2.left_trigger / 2);

        }else if (gamepad2.right_trigger > deadZone) {
               // liftMotor.setPower(-(gamepad2.right_trigger / 2));

            } else{
              //  liftMotor.setPower(0);       // this is the 'ABORT!!!!' point.... if we dont meet any of the above stop the motor!!!! otherwise it keeps moving
            }



        if (gamepad2.right_bumper){
            turnRight = true;
            turnLeft = false;

        } else if (gamepad2.left_bumper){
            turnRight = false;
            turnLeft = true;

        }

        if (turnLeft && !left){
            //TURN THE MOTOR LEFT USING ENCODERS
            right = false;
            left = true;

        } else if (turnRight && !right){
            //TURN THE MOTOR RIGHT USING ENCODERS
            right = true;
            left = false;

         }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    }
        @Override
        public void stop () { //You'll never find all of these
        }
    }

