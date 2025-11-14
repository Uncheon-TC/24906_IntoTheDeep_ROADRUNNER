package org.firstinspires.ftc.teamcode.drive.FTC2025Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "color sensor", group = "2024-2025 Test OP")
public class TestBenchColor extends LinearOpMode {

    // 색상 센서 객체
    private ColorSensor colorSensor;

    // Green/Purple 판단 기준 (필요하면 숫자 조절)
    private final int GREEN_DIFF = 15;   // Green과 다른 색 차이 (조금 더 큰 차이를 요구)
    private final int PURPLE_DIFF = 20;  // Blue가 가장 높을 때만 Purple

    @Override
    public void runOpMode() {

        // 하드웨어 맵에서 색상 센서를 가져옴
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // LED 켜기 (센서에 따라 자동작동일 수도 있음)
        try {
            colorSensor.enableLed(true);
        } catch (Exception ignored) {}

        waitForStart();

        while (opModeIsActive()) {

            // 감지된 색상의 RGB 값을 얻음
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // 감지된 색 판별
            String detectedColor = "UNKNOWN";

            if (isGreen(red, green, blue)) {
                detectedColor = "GREEN";
            }
            else if (isPurple(red, green, blue)) {
                detectedColor = "PURPLE";
            }

            // Telemetry 출력
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            sleep(100);
        }
    }

    // GREEN 판별: green이 red & blue보다 확실하게 높고, 차이가 충분히 클 때만 GREEN
    private boolean isGreen(int red, int green, int blue) {
        return (green > red && green > blue) &&
                (green - red > GREEN_DIFF) &&
                (green - blue > GREEN_DIFF)&&(green-red>100);
    }

    // PURPLE 판별: blue가 가장 크고, blue와 red의 차이가 충분히 큰 경우 PURPLE
    private boolean isPurple(int red, int green, int blue) {
        return (blue > red && blue > green) &&
                ((blue - red > PURPLE_DIFF) || (blue - green > PURPLE_DIFF));
    }
}
