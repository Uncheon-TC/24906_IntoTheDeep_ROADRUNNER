package org.firstinspires.ftc.teamcode.drive.FTC2025Test;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//TODO: find drive motor Direction and setting IMU position  ->  DONE

//TODO: install entire servo and tuning

@TeleOp (name = "미국test", group = "2024-2025 Test OP")

public class test extends LinearOpMode {

    //pid settings
    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static double f = 0.001;

    public static int arm_target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AR;

    private Servo V_wristL;
    private Servo V_wristR;
    private Servo H_lengthL;
    private Servo H_lengthR;
    private Servo H_wristL;
    private Servo H_wristR;
    private Servo H_angleL;
    private Servo H_angleR;
    private Servo V_angleL;
    private Servo V_angleR;
    private Servo H_grip;
    private Servo V_grip;



    @Override
    public void runOpMode() throws InterruptedException {

        //pid setup
        controller = new PIDController(p,i,d);

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");
        AR.setDirection(DcMotorSimple.Direction.REVERSE);


        //declear motor

        DcMotor FrontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor FrontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor BackLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor BackRightMotor = hardwareMap.dcMotor.get("rightBack");

        //motor reverse
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU settings
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        /*
        DcMotor ArmLeft = hardwareMap.dcMotor.get("AL");
        DcMotor ArmRight = hardwareMap.dcMotor.get("AR");*/

        //V_wristR = hardwareMap.servo.get("V_wristR"); //Bucket Wrist right Servo
        V_wristL = hardwareMap.servo.get("V_wristL");
        V_wristR = hardwareMap.servo.get("V_wristR");//Bucket Wrist left Servo
        V_angleL = hardwareMap.servo.get("V_angleL");
        V_angleR = hardwareMap.servo.get("V_angleR");
        H_angleR = hardwareMap.servo.get("H_angleR");
        H_lengthL = hardwareMap.servo.get("H_lengthL");//Slide right Servo
        H_lengthR = hardwareMap.servo.get("H_lengthR");
        H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
        H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
        H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
        H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo
        H_grip = hardwareMap.servo.get("H_grip");
        V_grip = hardwareMap.servo.get("V_grip"); //vertical grip wrist

        H_wristL.setDirection(Servo.Direction.REVERSE);
        V_wristL.setDirection(Servo.Direction.REVERSE);

        H_lengthL.setDirection(Servo.Direction.REVERSE);

        H_angleL.setDirection(Servo.Direction.REVERSE);
        V_angleL.setDirection(Servo.Direction.REVERSE);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        waitForStart();



        //setting default var
        int arm_target = 0;

        boolean first_count = true;

        double V_Grip_OPEN = 0.43;
        double V_Grip_CLOSE = 0.65;

        double H_Grip_OPEN = 0.47;
        double H_Grip_CLOSE = 0.25;

        boolean HG_OPEN = true;
        boolean VG_OPEN = true;

        //TODO: find Horizon Griper value


        int clip_pick = 0;
        int High_backet = 2450;

        int High_chamber_hang = 800;


        //TODO: make rigging mechanism and find tick

        double V_wrist_L_pick = 0.5;
        double V_wrist_R_pick = 0.5;
        double V_wrist_L_hang = 0.97;
        double V_wrist_R_hang = 0.34;
        double V_wrist_L_trans = 0.92;
        double V_wrist_R_trans =0.94;
        double V_wrist_L_backet = 0.53;
        double V_wrist_R_backet = 0.55;

        double V_angle_pick = 0.16;
        double V_angle_up = 0.2;
        double V_angle_hang = 0.82;
        double V_angle_hang_down = 0.87;
        double V_angle_backet =0.73;
        double V_angle_trans_ready =0.54;
        double V_angle_trans = 0.43;



        int chamber_status = 0;



        double H_wristL_Ready = 0.45+0.03;
        double H_wristR_Ready = 0.45+0.03;

        double H_wrist_L_hide = 0.5;
        double H_wrist_R_hide = 0.5;
        double H_wrist_L_trans = 1;
        double H_wrist_R_trans = 1;

        double H_angleL_back = 0.47;
        double H_angleR_back = 0.47;


        double H_wristL_pickup = 0.5;
        double H_wristR_pickup = 0.5;
        double H_length_IN = 0.4;
        //double H_length_L_IN = 0.62;
        //double H_length_R_IN = 0.60;
        double H_length_OUT = 0.1;
        //double H_length_L_OUT = 0.32;
        //double H_length_R_OUT = 0.30;
        double H_length_L_trans = 0.60;
        double H_length_R_trans = 0.58;

        double H_angle_Ready = 0.50;
        double H_angle_pickup = 0.68;
        double H_angle_hide = 0.76;
        double H_angle_trans = 0.46;

        double H_wristL_target = 0;
        double H_wristR_target = 0;

        double interval = 0.075;

        /*Gamepad.LedEffect sample_RED = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 100)
                .setRepeating(true)
                .build();

        Gamepad.LedEffect sample_BLUE = new Gamepad.LedEffect.Builder()
                .addStep(0, 0, 1, 100)
                .setRepeating(true)
                .build();

        Gamepad.LedEffect sample_YELLOW = new Gamepad.LedEffect.Builder()
                .addStep(1, 1, 0, 100)
                .setRepeating(true)
                .build();*/


        while (opModeIsActive()) {

            if (isStopRequested()) return;  //check stop button pushed?

            if (first_count) {
                //write code
                arm_target = clip_pick;

                V_grip.setPosition(V_Grip_OPEN);
                H_grip.setPosition(H_Grip_OPEN);

                V_wristL.setPosition(V_wrist_L_pick);
                V_wristR.setPosition(V_wrist_R_pick);
                V_angleL.setPosition(V_angle_up);
                V_angleR.setPosition(V_angle_up);

                H_angleL.setPosition(H_angle_Ready);
                H_angleR.setPosition(H_angle_Ready);
                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);

                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);




                //angle_control(H_angle_trans);
                //wrist_control(H_wristL_POS180, H_wristR_POS180);

                first_count = false;
            }

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



            //
            double y = -gamepad1.left_stick_y;  //y value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);   //slow mechanism, change 0.8 to slow leveling

            //
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX - rx) / denominator) * slow;
            double backLeftPower = ((rotY - rotX - rx) / denominator) * slow;
            double frontRightPower = ((rotY - rotX + rx) / denominator) * slow;
            double backRightPower = ((rotY + rotX + rx) / denominator) * slow;

            FrontLeftMotor.setPower(frontLeftPower);
            BackLeftMotor.setPower(backLeftPower);
            FrontRightMotor.setPower(frontRightPower);
            BackRightMotor.setPower(backRightPower);


            //continue main coading

            if (rising_edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up)) {
                H_lengthL.setPosition(H_length_OUT);
                H_lengthR.setPosition(H_length_OUT);
                H_angleL.setPosition(H_angle_Ready+0.01);
                H_angleR.setPosition(H_angle_Ready+0.01);



                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);

                //HG_OPEN = true;
            }

            if (rising_edge(currentGamepad1.x, previousGamepad1.x)) {
                    H_angleL.setPosition(H_angle_pickup);
                H_angleR.setPosition(H_angle_pickup);



            }

            if (rising_edge(currentGamepad1.left_bumper, previousGamepad1.left_bumper)) {
                H_grip.setPosition(H_Grip_CLOSE);
                H_angleL.setPosition(H_angle_pickup+0.05);
                H_angleR.setPosition(H_angle_pickup+0.05);
                H_wristL.setPosition(H_wristL_Ready+0.03);
                H_wristR.setPosition(H_wristR_Ready+0.03);

            }

            if (rising_edge(currentGamepad1.right_bumper, previousGamepad1.right_bumper)) {
                H_grip.setPosition(H_Grip_OPEN);
                H_angleL.setPosition(H_angle_pickup);
                H_angleR.setPosition(H_angle_pickup);
                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);

            }

            if (rising_edge(currentGamepad1.y, previousGamepad1.y)) {

                H_angleL.setPosition(H_angle_trans);
                H_angleR.setPosition(H_angle_trans);

                H_wristL.setPosition(H_wrist_L_trans);
                H_wristR.setPosition(H_wrist_R_trans);

                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);

                V_angleL.setPosition(V_angle_trans);
                V_angleR.setPosition(V_angle_trans);

                V_wristL.setPosition(V_wrist_L_trans);
                V_wristR.setPosition(V_wrist_R_trans);



                //VG_OPEN = true;
            }

            if (rising_edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down)) {
                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);
                H_angleL.setPosition(H_angleL_back+0.02);
                H_angleR.setPosition(H_angleR_back+0.02);
                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);
                V_angleL.setPosition(V_angle_up);
                V_angleL.setPosition(V_angle_up);


            }
            if (rising_edge(currentGamepad1.a, previousGamepad1.a)) {
                arm_target = clip_pick;
                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);
                V_angleL.setPosition(V_angle_pick);
                V_angleR.setPosition(V_angle_pick);
                V_wristL.setPosition(V_wrist_L_pick);
                V_wristR.setPosition(V_wrist_R_pick);
                H_angleL.setPosition(H_angle_Ready+0.01);
                H_angleR.setPosition(H_angle_Ready+0.01);



                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);
                //HG_OPEN = true;
                //VG_OPEN = false;
            }

            if (rising_edge(currentGamepad1.b, previousGamepad1.b)) {
                arm_target = High_chamber_hang;
                V_angleL.setPosition(V_angle_hang);
                V_angleR.setPosition(V_angle_hang);
                V_wristL.setPosition(V_wrist_L_hang);
                V_wristR.setPosition(V_wrist_R_hang);

                //HG_OPEN = true;
                //VG_OPEN = false;
            }

            if (rising_edge(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
                H_wristL_target = H_wristL.getPosition() + interval;
                H_wristR_target = H_wristR.getPosition() - interval;

                if (H_wristL_target > 1) {
                    H_wristL_target = 1;
                }

                if (H_wristR_target < 0) {
                    H_wristR_target = 0;
                }

                wrist_control(H_wristL_target, H_wristR_target);
            }

            if (rising_edge(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
                H_wristL_target = H_wristL.getPosition() - interval;
                H_wristR_target = H_wristR.getPosition() + interval;

                if (H_wristL_target < 0) {
                    H_wristL_target = 0;
                }

                if (H_wristR_target > 1) {
                    H_wristR_target = 1;
                }

                wrist_control(H_wristL_target, H_wristR_target);
            }

            if (gamepad2.left_bumper) {
                V_grip.setPosition(V_Grip_CLOSE);
            }
            if (gamepad2.right_bumper) {
                V_grip.setPosition(V_Grip_OPEN);
            }
/*
            if (rising_edge(currentGamepad2.left_bumper, previousGamepad2.left_bumper)) {
                V_grip.setPosition(V_Grip_CLOSE);

            }

            if (rising_edge(currentGamepad2.right_bumper, previousGamepad2.right_bumper)) {
                V_grip.setPosition(V_Grip_OPEN);

            }
*/
            if (rising_edge(currentGamepad2.a, previousGamepad2.a)) {
                arm_target = clip_pick;
                V_angleL.setPosition(V_angle_trans);
                V_angleR.setPosition(V_angle_trans);
                V_wristL.setPosition(V_wrist_L_trans);
                V_wristR.setPosition(V_wrist_R_trans);
                V_grip.setPosition(V_Grip_OPEN);
                H_angleL.setPosition(H_angle_Ready+0.01);
                H_angleR.setPosition(H_angle_Ready+0.01);



                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);
            }


            if (rising_edge(currentGamepad2.b, previousGamepad2.b)) {
                arm_target = High_backet;
                V_angleL.setPosition(V_angle_backet);
                V_angleR.setPosition(V_angle_backet);
                V_wristL.setPosition(V_wrist_L_backet);
                V_wristR.setPosition(V_wrist_R_backet);
                H_grip.setPosition(H_Grip_OPEN);


            }

            if (rising_edge(currentGamepad2.x, previousGamepad2.x)) {

                if (chamber_status == 0) {

                   // arm_target = High_chamber;

                    chamber_status = 1;

                } else if (chamber_status == 1) {

                    arm_target = clip_pick;
                    V_angleL.setPosition(V_angle_trans);
                    V_angleR.setPosition(V_angle_trans);
                    chamber_status = 0;

                }

            }

            if (rising_edge(currentGamepad2.y, previousGamepad2.y)) {
                V_angleR.setPosition(V_angle_hang_down);
                V_angleL.setPosition(V_angle_hang_down);
                V_wristL.setPosition(V_wrist_L_hang);
                V_wristR.setPosition(V_wrist_R_hang);

            }

            if (gamepad2.right_stick_button){
                resetEncoder(AL);
                arm_target = 0;
            }
            if (rising_edge(currentGamepad2.dpad_left, previousGamepad2.dpad_left))
            {
                H_wristL_target = H_wristL.getPosition() - interval;
                H_wristR_target = H_wristR.getPosition() + interval;

                if (H_wristL_target < 0) {
                    H_wristL_target = 0;
                }

                if (H_wristR_target > 1) {
                    H_wristR_target = 1;
                }

                wrist_control(H_wristL_target, H_wristR_target);
            }






            //telemetry settings
            telemetry.addData("ArmPos ", ArmPos);
            telemetry.addData("Target Pos ", arm_target);
            telemetry.addLine();
            //telemetry.addData("HG", HG_OPEN);
            //telemetry.addData("VG", VG_OPEN);
            telemetry.update();  //update telemetry, end of line
        }

    }

    private boolean rising_edge(boolean currentButtonState, boolean previousButtonState) {
        return currentButtonState && !previousButtonState;
    }

    private void wrist_control(double L, double R) {
        H_wristL.setPosition(L);
        H_wristR.setPosition(R);
    }

    private void angle_control(double target) {
        H_angleL.setPosition(target);
        H_angleR.setPosition(target);
    }

    private void resetEncoder(DcMotorEx AL) {
        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
