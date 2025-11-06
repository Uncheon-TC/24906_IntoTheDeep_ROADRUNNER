package org.firstinspires.ftc.teamcode.drive.FTC2025Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "shoot", group = "2024-2025 Test OP")
public class Maindrive_test2 extends LinearOpMode {

    private DcMotor SL;
    private DcMotor SR;
    private DcMotor GT;
    private Servo servo_L;
    private Servo servo_R;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {

        // 🔹 모터 초기화
        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");
        GT = hardwareMap.dcMotor.get("GT");

        // 🔹 방향 설정
        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);
        GT.setDirection(DcMotorSimple.Direction.FORWARD);

        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);

        // 🔹 서보 초기화
        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");

        servo_L.setDirection(Servo.Direction.FORWARD);
        servo_R.setDirection(Servo.Direction.REVERSE);

        // 🔹 기본값
        double power = 0.7;
        double step = 0.02;
        double servoDown = 0.0;   // 0도
        double servoUp = 0.36;    // 60도 정도
        boolean shooterOn = false;
        boolean intakeOn = false;

        // 🔹 init 시 서보 0도로 이동
        servo_L.setPosition(servoDown);
        servo_R.setPosition(servoDown);

        telemetry.addLine("=== Ready to Start ===");
        telemetry.addLine("서보가 0도 위치로 초기화되었습니다.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // 🔹 D-pad ↑↓로 쏘는 모터 속도 조절
            if (rising_edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up)) {
                power = Math.min(1.0, power + step);
            }

            if (rising_edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down)) {
                power = Math.max(0.0, power - step);
            }

            // 🔹 X 버튼 → 쏘는 모터 켜기
            if (rising_edge(currentGamepad1.x, previousGamepad1.x)) {
                shooterOn = true;
            }

            // 🔹 Y 버튼 → 쏘는 모터 끄기
            if (rising_edge(currentGamepad1.y, previousGamepad1.y)) {
                shooterOn = false;
            }

            // 🔹 쏘는 모터 동작
            if (shooterOn) {
                SL.setPower(power);
                SR.setPower(power);
            } else {
                SL.setPower(0);
                SR.setPower(0);
            }

            // 🔹 GT (흡입 모터) 제어: A = 켜기, B = 끄기
            if (rising_edge(currentGamepad1.a, previousGamepad1.a)) {
                GT.setPower(0.6);
            }

            if (rising_edge(currentGamepad1.b, previousGamepad1.b)) {
                GT.setPower(0);
            }

            // 🔹 서보 제어: D-pad → = 60도 / D-pad ← = 0도
            if (rising_edge(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
                servo_L.setPosition(servoUp);
                servo_R.setPosition(servoUp);
            }

            if (rising_edge(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
                servo_L.setPosition(servoDown);
                servo_R.setPosition(servoDown);
            }

            // 🔹 Telemetry 표시
            telemetry.addData("Shooter Power", "%.2f", power);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("GT Power", GT.getPower());
            telemetry.addData("Servo_L Pos", "%.2f", servo_L.getPosition());
            telemetry.addData("Servo_R Pos", "%.2f", servo_R.getPosition());
            telemetry.update();
        }

        // 🔹 정지 시 초기화
        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);
        servo_L.setPosition(servoDown);
        servo_R.setPosition(servoDown);
    }

    // 🔹 버튼 눌림 감지 (rising edge)
    private boolean rising_edge(boolean currentButtonState, boolean previousButtonState) {
        return currentButtonState && !previousButtonState;
    }
}
