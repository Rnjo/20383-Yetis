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

package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bina;
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


public class BlueAllianceLeftWithDriveConstants extends Bina {

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
                leftRear.setTargetPosition(1);
                leftFront.setTargetPosition(1);
                rightRear.setTargetPosition(1);
                rightRear.setTargetPosition(1);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(0.7);
                leftFront.setPower(0.7);
                sleep(1500);
                rightRear.setTargetPosition(1380);
                leftRear.setTargetPosition(1380);
                leftFront.setTargetPosition(1380);
                rightFront.setTargetPosition(1380);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(0.7);
                leftFront.setPower(0.7);
                sleep(2000);
                leftRear.setTargetPosition(1);
                leftFront.setTargetPosition(1);
                rightRear.setTargetPosition(1);
                rightRear.setTargetPosition(1);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(-0.7);
                leftFront.setPower(-0.7);
                sleep(1500);

            }
            case RIGHT: {
                rightRear.setTargetPosition(1044);
                leftRear.setTargetPosition(1044);
                leftFront.setTargetPosition(1044);
                rightFront.setTargetPosition(1044);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(0.7);
                leftFront.setPower(0.7);
                sleep(2000);
                leftRear.setTargetPosition(972);
                leftFront.setTargetPosition(1154);
                rightRear.setTargetPosition(1154);
                rightRear.setTargetPosition(972);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(-0.7);
                leftRear.setPower(-0.7);
                leftFront.setPower(0.7);
                sleep(1500);




            }
            case CENTER: {
                rightRear.setTargetPosition(1380);
                leftRear.setTargetPosition(1380);
                leftFront.setTargetPosition(1380);
                rightFront.setTargetPosition(1380);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRear.setPower(0.7);
                rightFront.setPower(0.7);
                leftRear.setPower(0.7);
                leftFront.setPower(0.7);
                sleep(2000);

            }


        }
    }

}