package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Basic Example", group = "Sensor")
public class LimelightBasicExample extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Limelight 장치 연결 (장치 이름은 Configuration에서 지정한 이름과 일치해야 함)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Limelight 파이프라인 선택 (0번 사용 중)
        limelight.pipelineSwitch(0);

        // Limelight 데이터 수신 시작
        limelight.start();

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();
        waitForStart();

        // 반복 루프: 로봇이 작동하는 동안 계속 실행
        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            // 시스템 상태 정보 출력
            telemetry.addData("Temp", "%.1fC", status.getTemp());
            telemetry.addData("CPU", "%.1f%%", status.getCpu());

            // 타겟 인식 여부와 좌표 출력
            if (result != null && result.isValid()) {
                telemetry.addData("Target Visible", true);
                telemetry.addData("tx (수평)", "%.2f", result.getTx());
                telemetry.addData("ty (수직)", "%.2f", result.getTy());
            } else {
                telemetry.addData("Target Visible", false);
            }

            telemetry.update();
        }

        // Limelight 데이터 수신 종료
        limelight.stop();
    }
}
