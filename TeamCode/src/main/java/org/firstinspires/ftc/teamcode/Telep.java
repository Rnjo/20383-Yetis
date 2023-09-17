package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.PipelineRecordingParameters;

//abhir
@TeleOp
public class Telep extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        DcMotor frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo claw = hardwareMap.get(Servo.class, "claw");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
//leftEncoder rightEncoder frontEncoder bore encoders
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
//test
            leftEncoder.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightEncoder.setPower(frontRightPower);
            frontEncoder.setPower(backRightPower);
//intake code
            if (gamepad2.a) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
//claw code
            if (gamepad2.b) {
                claw.setPosition(1);
            }else if (gamepad2.x){
                claw.setPosition(0);
            }


            telemetry.addData("FlPos", leftEncoder.getCurrentPosition());
            telemetry.addData("Rpos", leftRear.getCurrentPosition());
            telemetry.addData("frpos", rightEncoder.getCurrentPosition());
            telemetry.update();


        }
    }
}