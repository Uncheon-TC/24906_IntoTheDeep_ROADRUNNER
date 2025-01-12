package org.firstinspires.ftc.teamcode.drive.FTC2025Test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: merge to Maindrive_test

@TeleOp (name = "OPMODE PID TEST", group = "2024-2025 Test OP")
public class arm_pid_test extends LinearOpMode {

    //pid settings
    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static double f = 0.001;

    public static int arm_target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AR;

    @Override
    public void runOpMode() throws InterruptedException {

        //pid setup
        controller = new PIDController(p,i,d);

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");

        AR.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Servo V_wristL = hardwareMap.servo.get("V_wristL");
        Servo V_grip = hardwareMap.servo.get("V_grip");

        double wrist = 0;
        double grip = 0;

        waitForStart();





        while (opModeIsActive()) {

            if (isStopRequested()) return;

            //pid calculate
            controller.setPID(p,i,d);


            int ArmPos = AL.getCurrentPosition();
            double pid = controller.calculate(ArmPos, arm_target);
            double ff = Math.cos(Math.toRadians(arm_target / ticks_in_degree))*f;

            double ArmPower = pid + ff;

            AL.setPower(ArmPower);
            AR.setPower(ArmPower);

            //
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            //
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                if (arm_target >= 4100) {
                    arm_target = 4200;
                } else {
                    arm_target = arm_target + 100;
                }
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                if (arm_target <= 100) {
                    arm_target = 0;
                } else {
                    arm_target = arm_target - 100;
                }

            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                wrist = wrist + 0.01;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                wrist = wrist - 0.01;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                wrist = wrist + 0.1;
                V_wristL.setPosition(wrist);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                wrist = wrist - 0.1;
                V_wristL.setPosition(wrist);
            }


            telemetry.addData("target ", arm_target);
            telemetry.addData("pos ", ArmPos);
            telemetry.addData("power ", ArmPower);
            telemetry.addLine();
            telemetry.addData("grip", wrist);
            telemetry.addData("angle", grip);

            telemetry.update();

        }
    }
}
