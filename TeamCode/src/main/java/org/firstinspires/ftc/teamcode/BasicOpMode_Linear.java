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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor motorIntake;
    private DcMotor slideLeft;
    private DcMotor slideRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class,   "back_left_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        motorIntake = hardwareMap.get(DcMotor.class, "motor_intake");
        slideLeft = hardwareMap.get(DcMotor.class, "slide_left");
        slideRight = hardwareMap.get(DcMotor.class, "slide_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // frontLeft & backRight should be in reverse
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontleftPower;
            double frontrightPower;
            double backleftPower;
            double backrightPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            boolean intakeKicker = gamepad1.b;
            boolean intakeReverse = gamepad1.a;
            boolean slideUp = gamepad1.y;
            boolean slideDown = gamepad1.x;
            boolean isolateLeft = gamepad1.dpad_left;
            boolean isolateRight = gamepad1.dpad_right;


            //for intake movement
            if(intakeKicker){
                motorIntake.setPower(1);
            } else if(intakeReverse){
                motorIntake.setPower(-1);
            } else {
                motorIntake.setPower(0);
            }

            if(slideUp && !(isolateLeft || isolateRight) ){
                slideLeft.setPower(1);
                slideRight.setPower(1);
            } else if (slideDown && !(isolateLeft || isolateRight)){
                slideLeft.setPower(-1);
                slideRight.setPower(-1);
            } else {
                slideLeft.setPower(0);
                slideRight.setPower(0);
            }

            //Only use to adjust tension, not for use during gameplay:
            if(isolateLeft && slideUp){
                slideLeft.setPower(.5);
            } else if (slideDown && isolateLeft){
                slideLeft.setPower(-.5);
            } else {
                slideLeft.setPower(0);
            }

            if(isolateRight && slideUp){
                slideRight.setPower(.5);
            } else if (slideDown && isolateRight){
                slideRight.setPower(-.5);
            } else {
                slideRight.setPower(0);
            }


            frontleftPower   = Range.clip(drive + turn + strafe, -1.0, 1.0) ;
            frontrightPower  = Range.clip(drive - turn - strafe, -1.0, 1.0) ;
            backleftPower    = Range.clip(drive + turn - strafe, -1.0, 1.0) ;
            backrightPower   = Range.clip(drive - turn + strafe, -1.0, 1.0) ;

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontleftPower);
            frontRightDrive.setPower(frontrightPower);
            backLeftDrive.setPower(backleftPower);
            backRightDrive.setPower(backrightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)",
                    frontleftPower, frontrightPower, backleftPower, backrightPower);
            telemetry.update();
        }
    }
}
