package org.firstinspires.ftc.teamcode.drive.FTC2025Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "color sensor dual", group = "2024-2025 Test OP")
public class TestBenchColor extends LinearOpMode {

    // 색상 센서 객체
    private ColorSensor colorSensor_L;
    private ColorSensor colorSensor_R;

    // LED(Servo) 객체
    private Servo light_L;
    private Servo light_R;

    // Green/Purple 판단 기준
    private final int GREEN_DIFF = 15;
    private final int PURPLE_DIFF = 20;

    @Override
    public void runOpMode() {

        // 하드웨어 맵에서 센서/서보 가져오기
        colorSensor_L = hardwareMap.get(ColorSensor.class, "colorSensor_L");
        colorSensor_R = hardwareMap.get(ColorSensor.class, "colorSensor_R");

        light_L = hardwareMap.servo.get("light_L");
        light_R = hardwareMap.servo.get("light_R");

        try {
            colorSensor_L.enableLed(true);
        } catch (Exception ignored) {}
        try {
            colorSensor_R.enableLed(true);
        } catch (Exception ignored) {}

        waitForStart();

        while (opModeIsActive()) {

            // 좌측 센서 감지
            int red_L = colorSensor_L.red();
            int green_L = colorSensor_L.green();
            int blue_L = colorSensor_L.blue();
            String detectedColor_L = detectColor(red_L, green_L, blue_L);
            setServo(light_L, detectedColor_L);

            // 우측 센서 감지
            int red_R = colorSensor_R.red();
            int green_R = colorSensor_R.green();
            int blue_R = colorSensor_R.blue();
            String detectedColor_R = detectColor(red_R, green_R, blue_R);
            setServo(light_R, detectedColor_R);

            // Telemetry
            telemetry.addData("Left Sensor", detectedColor_L + " [R:%d G:%d B:%d]", red_L, green_L, blue_L);
            telemetry.addData("Right Sensor", detectedColor_R + " [R:%d G:%d B:%d]", red_R, green_R, blue_R);
            telemetry.update();

            sleep(100);
        }
    }

    // 색 판별
    private String detectColor(int red, int green, int blue) {
        if (isGreen(red, green, blue)) {
            return "GREEN";
        } else if (isPurple(red, green, blue)) {
            return "PURPLE";
        } else {
            return "UNKNOWN";
        }
    }

    // Servo 위치 설정
    private void setServo(Servo servo, String detectedColor) {
        switch (detectedColor) {
            case "GREEN":
                servo.setPosition(0.5);
                break;
            case "PURPLE":
                servo.setPosition(0.722);
                break;
            default:
                servo.setPosition(0);
                break;
        }
    }

    // GREEN 판별
    private boolean isGreen(int red, int green, int blue) {
        return (green > red && green > blue) &&
                (green - red > GREEN_DIFF) &&
                (green - blue > GREEN_DIFF) &&
                (green - red > 100);
    }

    // PURPLE 판별
    private boolean isPurple(int red, int green, int blue) {
        return (blue > red && blue > green) &&
                ((blue - red > PURPLE_DIFF) || (blue - green > PURPLE_DIFF));
    }
}
