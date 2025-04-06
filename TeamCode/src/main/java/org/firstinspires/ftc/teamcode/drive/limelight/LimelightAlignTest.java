package org.firstinspires.ftc.teamcode.drive.limelight;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Limelight Align Test", group = "Autonomous")
public class LimelightAlignTest extends LinearOpMode {

    private Limelight3A limelight;
    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;
    public static double f = 0.001;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-6.7, 62.4, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        controller = new PIDController(p, i, d);

        // Limelight 초기화
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // 파란색 인식용 파이프라인 번호
        limelight.start();

        waitForStart();

        TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(5, 31), Math.PI / 2)

                // ⭐️ 정렬 동작 추가
                .stopAndAdd(() -> alignWithBlueObject(drive, limelight));

        // 전체 Trajectory 실행
        Actions.runBlocking(traj.build());

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            telemetry.update();
        }
    }

    // ⭐️ Limelight로 파란색 물체 중심 정렬 함수
    public void alignWithBlueObject(MecanumDrive drive, Limelight3A limelight) {
        double startTime = getRuntime();
        double timeout = 5.0; // 최대 동작 시간
        double kP = 0.025;    // 회전 속도 비례 제어 계수
        double txThreshold = 1.5; // 중앙 정렬 허용 오차

        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("tx", tx);

                if (Math.abs(tx) < txThreshold) {
                    telemetry.addLine("🎯 Aligned to blue object!");
                    break;
                }

                double turn = -kP * tx;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), turn));  // ✅ 수정된 부분
            } else {
                telemetry.addLine("❌ No target detected");
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
            }

            telemetry.update();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }
}
