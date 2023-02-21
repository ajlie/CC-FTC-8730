package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */

@TeleOp(name = "Telemetry", group = "MecanumBot")
public class Telemetry extends LinearOpMode {

    DcMotor m1, m2, m3, m4, mlift;
    Servo scissorsLeft, scissorsRight;
    public void runOpMode(){
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        mlift = hardwareMap.dcMotor.get("lift_motor");
        //scissorsLeft = hardwareMap.servo.get("scissorsLeft");
        //scissorsRight = hardwareMap.servo.get("scissorsRight");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        //scissorsLeft.setDirection(Servo.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
//        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
//        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
//        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        //ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            //set buttons to a variable
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            //calculate inputs from joystick
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            if (gamepad1.right_bumper) {
                //set power for each motor
                m1.setPower(p1);
                m2.setPower(p2);
                m3.setPower(p3);
                m4.setPower(p4);
            } else if (gamepad1.left_bumper) {
                m1.setPower(p1/4);
                m2.setPower(p2/4);
                m3.setPower(p3/4);
                m4.setPower(p4/4);
            } else {
                m1.setPower(p1/2);
                m2.setPower(p2/2);
                m3.setPower(p3/2);
                m4.setPower(p4/2);
            }
            if (gamepad2.dpad_up) {
                //armMotor.setTargetPosition(tierOne);
                mlift.setPower(1);
            } else {
                mlift.setPower(0);
            }

            if (gamepad2.dpad_down) {
                //armMotor.setTargetPosition(tierTwo);
                mlift.setPower(-1);
            } else {
                mlift.setPower(0);
            }
        }

        /*
        if (gamepad2.right_bumper) {
            scissorsLeft.setPosition(0.4);
            scissorsRight.setPosition(0.4);
        }
        else if (gamepad2.left_bumper) {
            scissorsLeft.setPosition(0);
            scissorsRight.setPosition(0);
        }
        */



        //telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

//        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                m3.getCurrentPosition(), m4.getCurrentPosition());
        telemetry.update();


    }
}