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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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


public class RedAllianceLeft extends Bina {

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


        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
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

        drive.setPoseEstimate(new Pose2d(64.25, 15.6, 270));
// left movements
        Trajectory moveToTapeLeft = drive.trajectoryBuilder(new Pose2d(64.25, 15.6, 270))
                .lineToConstantHeading(new Vector2d(30, 56.4))
                .build();

        Trajectory moveToBoardLeft = drive.trajectoryBuilder(moveToTapeLeft.end())
                .lineToConstantHeading(new Vector2d(44, 39))
                .build();

        Trajectory Park = drive.trajectoryBuilder(moveToBoardLeft.end())
                .back(4)
                .build();

//middle movements
        Trajectory moveToTapeMiddle = drive.trajectoryBuilder(new Pose2d(64.25, 15.6, 270))
                .lineToConstantHeading(new Vector2d(24, 20.5))
                .build();


        Trajectory moveToBoardMiddle = drive.trajectoryBuilder(moveToTapeMiddle.end())
                .lineToConstantHeading(new Vector2d(44, 39))
                .build();

//right movements

        Trajectory moveToTapeRight = drive.trajectoryBuilder(new Pose2d(64.25, 15.6, 270))
                .lineToConstantHeading(new Vector2d(30, 8.5))
                .build();

        Trajectory moveToBoardRight = drive.trajectoryBuilder(moveToTapeRight.end())
                .lineToConstantHeading(new Vector2d(44, 39))
                .build();


        switch (snapshotAnalysis) {
            case LEFT: {
                drive.followTrajectory(moveToTapeLeft);
                intake1.setPower(-0.4);
                intake2.setPower(-0.4);
                sleep(1000);
                intake1.setPower(0);
                intake2.setPower(0);
                drive.followTrajectory(moveToBoardLeft);
                lift.setTargetPosition(lift_max_position);
                sleep(1000);
                gates.setPower(-0.1);
                sleep(2000);
                gates.setPower(0);
                lift.setTargetPosition(lift_min_position);
                drive.followTrajectory(Park);


            }
            case RIGHT: {
                drive.followTrajectory(moveToTapeRight);
                intake1.setPower(-0.4);
                intake2.setPower(-0.4);
                sleep(1000);
                intake1.setPower(0);
                intake2.setPower(0);
                drive.followTrajectory(moveToBoardRight);
                lift.setTargetPosition(lift_max_position);
                sleep(1000);
                gates.setPower(-0.1);
                sleep(2000);
                gates.setPower(0);
                lift.setTargetPosition(lift_min_position);
                drive.followTrajectory(Park);




            }
            case CENTER: {
                drive.followTrajectory(moveToTapeMiddle);
                intake1.setPower(-0.4);
                intake2.setPower(-0.4);
                sleep(1000);
                intake1.setPower(0);
                intake2.setPower(0);
                drive.followTrajectory(moveToBoardMiddle);
                lift.setTargetPosition(lift_max_position);
                sleep(1000);
                gates.setPower(-0.1);
                sleep(2000);
                gates.setPower(0);
                lift.setTargetPosition(lift_min_position);
                drive.followTrajectory(Park);
            }
        }
    }
}

