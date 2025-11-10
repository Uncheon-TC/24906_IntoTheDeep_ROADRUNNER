package org.firstinspires.ftc.teamcode.drive.FTC2025Test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "shoot", group = "2024-2025 Test OP")
public class Maindrive_test2 extends LinearOpMode {


    private DcMotor SL;
    private DcMotor SR;
    private DcMotor GT;


    private Servo servo_L;
    private Servo servo_R;

    @Override
    public void runOpMode() throws InterruptedException {


        SL = hardwareMap.dcMotor.get("SL");
        SR = hardwareMap.dcMotor.get("SR");
        GT = hardwareMap.dcMotor.get("GT");

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.FORWARD);
        GT.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor FrontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor FrontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor BackLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor BackRightMotor = hardwareMap.dcMotor.get("BR");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");


        servo_L.setDirection(Servo.Direction.REVERSE);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        boolean shooterOn = false;
        double power = 0.78;
        double step = 0.02;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * slow;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * slow;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * slow;
            double backRightPower = ((rotY + rotX - rx) / denominator) * slow;


            FrontLeftMotor.setPower(frontLeftPower);
            BackLeftMotor.setPower(backLeftPower);
            FrontRightMotor.setPower(frontRightPower);
            BackRightMotor.setPower(backRightPower);


            if (rising_edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up))
                power = Math.min(1.0, power + step);
            if (rising_edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down))
                power = Math.max(0.0, power - step);

            if (rising_edge(currentGamepad1.x, previousGamepad1.x))
                shooterOn = true;
            if (rising_edge(currentGamepad1.y, previousGamepad1.y))
                shooterOn = false;

            if (shooterOn) {
                SL.setPower(power);
                SR.setPower(power);
            } else {
                SL.setPower(0);
                SR.setPower(0);
            }


            if (rising_edge(currentGamepad1.a, previousGamepad1.a))
                GT.setPower(0.6);
            if (rising_edge(currentGamepad1.b, previousGamepad1.b))
                GT.setPower(0);



            if (gamepad1.right_bumper) {
                servo_R.setPosition(0.1);
            } else {
                servo_R.setPosition(0.5);
            }


            if (gamepad1.left_bumper) {
                servo_L.setPosition(0.1);
            } else {
                servo_L.setPosition(0.5);
            }


            telemetry.addData("Shooter Power", "%.2f", power);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("GT Power", GT.getPower());
            telemetry.addData("servo_pos", servo_L.getPosition());
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }


        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);
    }

    private boolean rising_edge(boolean current, boolean previous) {
        return current && !previous;
    }
}
