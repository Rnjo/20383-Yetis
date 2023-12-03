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


public class BlueAllianceLeft extends Bina {

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


        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(-72, 12, 0))
                .forward(40)
                .build();
        Trajectory LeftTurn = drive.trajectoryBuilder(new Pose2d(-32, 12, 0))
                 .lineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(90)))
                .forward(3)
                .build();
        Trajectory LeftTurnBack = drive.trajectoryBuilder(new Pose2d(-32, 12, 90))
                .back(3)
                .lineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(0)))
                .build();
        Trajectory RightTurn = drive.trajectoryBuilder(new Pose2d(-32, 12, 0))
                .lineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(-90)))
                .forward(3)
                .build();
        Trajectory Left = drive.trajectoryBuilder(new Pose2d(-36, 36, 270))
                .strafeRight(6)
                .build();
        Trajectory Right = drive.trajectoryBuilder(new Pose2d(-36, 36, 270))
                .strafeLeft(6)
                .build();
        Trajectory RightTurnBack = drive.trajectoryBuilder(new Pose2d(-32, 12, -90))
                .back(3)
                .lineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(0)))
                .build();
        Trajectory turnyThingy = drive.trajectoryBuilder(new Pose2d(-32, 12, 0))
                .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(-90)))
                .build();
        Trajectory imFinnaMoveBack = drive.trajectoryBuilder(new Pose2d(-36, 36, 0))
                .back(12)
                .build();



        drive.followTrajectory(myTrajectory);
        switch (snapshotAnalysis) {
            case LEFT: {
                drive.followTrajectory(LeftTurn);
                gates.setPower(-1);
                intake1.setPower(-1);
                intake2.setPower(-1);
                sleep(1000);
                drive.followTrajectory(LeftTurnBack);
                drive.followTrajectory(Left);

            }
            case RIGHT: {
                drive.followTrajectory(RightTurn);
                gates.setPower(-1);
                intake1.setPower(-1);
                intake2.setPower(-1);
                sleep(1000);
                drive.followTrajectory(RightTurnBack);
                drive.followTrajectory(Right);

            }
            case CENTER: {
                gates.setPower(-1);
                intake1.setPower(-1);
                intake2.setPower(-1);
            }
            drive.followTrajectory(turnyThingy);
            arm1.setPosition(1);
            arm2.setPosition(1);
            sleep(1000);
            gates.setPower(-1);
            sleep(1000);
            gates.setPower(0);
            arm1.setPosition(0);
            arm2.setPosition(0);
            drive.followTrajectory(imFinnaMoveBack);


        }
    }
    }

