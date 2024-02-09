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

//abhir auton
/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous


public class BlueAllianceRightWithDriveEncoders extends myDriveTrain {

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
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
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
                leftAndRight(5.5);
                sleep(1000);
                toAndFro(32);
                sleep(2500);
                turn(false);
                sleep(1000);
                intake1.setPower(-0.3);     // outtake first pixel onto spike mark
                sleep(3000);
                intake1.setPower(0);
                turn(false);
                sleep(2000);
                turn(false);
                sleep(2000);
                leftAndRight(-50);
                sleep(6000);
                toAndFro(-10);
                sleep(2000);
                leftAndRight(-40);
                sleep(5000);
                turn(false);
                sleep(1000);
                break;
            }
            case RIGHT: {
                leftAndRight(5);
                sleep(1000);
                toAndFro(23);
                sleep(2000);
                leftAndRight(27);  // strafe positive goes right , neg goes left
                sleep(3000);
                turn(false);             // true turns right, false turns left
                sleep(1500);
                leftAndRight(5);
                sleep(2000);
                intake1.setPower(-0.3);     // outtake first pixel onto spike mark
                sleep(3000);
                intake1.setPower(0);
                turn(false);
                sleep(2000);
                turn(false);
                sleep(2000);
                leftAndRight(-10);
                toAndFro(-15);
                sleep(2000);
                leftAndRight(-50);
                sleep(4000);
                toAndFro(-5);
                sleep(1000);
                leftAndRight(-30);
                sleep(4000);
                turn(false);
                sleep(1000);
                break;
            }
            case CENTER: {
                leftAndRight(5);
                sleep(1000);
                toAndFro(24);
                sleep(2000);
                intake1.setPower(-0.3);
                sleep(3000);
                intake1.setPower(0);
                turn(true);
                sleep(2000);
                leftAndRight(-50);
                sleep(4000);
                toAndFro(-20);
                leftAndRight(-40);
                sleep(4000);
                turn(false);
                sleep(1000);
                break;

            }


        }
    }

}