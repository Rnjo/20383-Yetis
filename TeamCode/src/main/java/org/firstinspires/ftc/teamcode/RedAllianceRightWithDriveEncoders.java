/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.PowerplayblueDeterminationExample;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous

//abhir auton
public class RedAllianceRightWithDriveEncoders extends myDriveTrain {

    OpenCvWebcam webcam;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline pipeline;
    PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = PowerplayblueDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /*
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PowerplayblueDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */



        waitForStart();
        lift = hardwareMap.get(DcMotor.class, "lift");
        gates = hardwareMap.get(CRServo.class, "gates");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) rightRear).setTargetPositionTolerance(5);
        ((DcMotorEx) leftRear).setTargetPositionTolerance(5);
        ((DcMotorEx) rightFront).setTargetPositionTolerance(5);
        ((DcMotorEx) leftFront).setTargetPositionTolerance(5);


        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(-72, 12, 0));


        switch (snapshotAnalysis) {
            case LEFT: {
            toAndFro(30);
            sleep(3000);
            turn(false);
            intake1.setPower(-0.3);     // outtake first pixel onto spik
            sleep(2000);
            intake1.setPower(-0.3);
            toAndFro(-33);
            sleep(2000);
            lift.setTargetPosition(-1900);
            lift.setPower(1);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            arm1.setPosition(1);
            arm2.setPosition(1);
            sleep(1000);
            gates.setPower(-1);
            sleep(1000);
            gates.setPower(0);
            toAndFro(3);
            sleep(1000);
            leftAndRight(-24);
            sleep(2000);
            toAndFro(-12);
            break;
            }
            case RIGHT: {
                toAndFro(23);
                sleep(2000);
                leftAndRight(25);  // strafe positive goes right , neg goes left
                sleep(3000);
                turn(false);             // true turns right, false turns left
                sleep(1500);
                leftAndRight(11);
                sleep(2000);
                intake1.setPower(-0.3);     // outtake first pixel onto spike mark
                sleep(2000);
                intake1.setPower(0);
                toAndFro(-16);     // go towards the board
                sleep(2000);
                leftAndRight(-17.5);  // move towards left of the board
                sleep(2000);
                leftFront.setTargetPosition(-120);
                rightFront.setTargetPosition(120);
                leftRear.setTargetPosition(-120);
                rightRear.setTargetPosition(120);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(0.7);
                leftFront.setPower(0.7);
                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                lift.setTargetPosition(-1450);
                lift.setPower(1);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                arm1.setPosition(1);
                arm2.setPosition(1);
                sleep(1000);
                gates.setPower(-1);
                sleep(1000);
                gates.setPower(0);
                toAndFro(3);     // go away from the board
                sleep(1000);
                /*lift.setTargetPosition(-200);
                lift.setPower(1);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                */leftAndRight(-22.);
                sleep(3000);
                toAndFro(-7);
                break;

            }
            case CENTER: {
               toAndFro(25);
                sleep(2000);
                leftAndRight(-4.66);
                sleep(2000);
                intake1.setPower(-0.7);
                intake2.setPower(-0.7);
                sleep(2000);
                intake1.setPower(0);
                intake2.setPower(0);
                turn(false);
                sleep(2000);
                toAndFro(-30.16);
                sleep(2000);
                sleep(1000);
                lift.setTargetPosition(-1450);
                lift.setPower(1);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                arm1.setPosition(1);
                arm2.setPosition(1);
                sleep(1000);
                gates.setPower(-1);
                sleep(1000);
                gates.setPower(0);
                toAndFro(3);
                sleep(2000);
                leftAndRight(20.54);
                toAndFro(-12);
                break;
            }


        }
    }

}