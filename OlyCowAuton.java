package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
    
@Autonomous
public class OlyCowAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        // Declaration
        ElapsedTime runtime = new ElapsedTime();
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("colorSensor");
        
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        
        runtime.reset();
        colorSensor.enableLed(true);
        
        //auton start
        
        forwards(0.5,1500,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
        
        //COLOR SENSOR            
        if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
            
            backwards(0.5,1300,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
            
            left(0.5,3000,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
            
            forwards(0.5,2500,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
        }
        
        else if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) {
            backwards(0.5,1300,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
        }
        
        else if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.blue()) {
            backwards(0.5,1300,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
            right(0.5,3000,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
            forwards(0.5,2500,motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft);
        }
        
    }
    
    public void forwards(double power,int time,DcMotor motorFrontRight,DcMotor motorFrontLeft,DcMotor motorBackRight,DcMotor motorBackLeft) {
        motorFrontRight.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(-power);
        sleep(time);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void backwards(double power,int time,DcMotor motorFrontRight,DcMotor motorFrontLeft,DcMotor motorBackRight,DcMotor motorBackLeft) {
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        sleep(time);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void right(double power,int time,DcMotor motorFrontRight,DcMotor motorFrontLeft,DcMotor motorBackRight,DcMotor motorBackLeft) {
        motorFrontRight.setPower(-power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);
        sleep(time);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void left(double power,int time,DcMotor motorFrontRight,DcMotor motorFrontLeft,DcMotor motorBackRight,DcMotor motorBackLeft) {
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(-power);
        sleep(time);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
}