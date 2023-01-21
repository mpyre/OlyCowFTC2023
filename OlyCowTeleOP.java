// packages
    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@TeleOp
public class OlyCowTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting telemetry
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        // Declarations
            ElapsedTime runtime = new ElapsedTime();
            DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
            DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
            DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            ColorSensor colorSensor = hardwareMap.colorSensor.get("colorSensor");
            DcMotor motorLinearSlide = hardwareMap.dcMotor.get("motorLinearSlide");
            Servo servoClaw = hardwareMap.servo.get("servoClaw");
        // Motor configuration
            motorLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        // RESET STUFF
            runtime.reset();
            colorSensor.enableLed(true);
            motorLinearSlide.setTargetPosition(500);

        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);

            // Claw Control
                if(gamepad2.a)
                {
                    servoClaw.setPosition(0.0);
                }
                else if (gamepad2.b)
                {
                    servoClaw.setPosition(1.0);
                }
            
            //Encoder Reset
                if (gamepad2.x) {
                    motorLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            
            //Linear Slide Controls 
                if (motorLinearSlide.getCurrentPosition() < 0) {
                    motorLinearSlide.setPower(0.1);
                }
                else if (motorLinearSlide.getCurrentPosition() > 2200){
                    motorLinearSlide.setPower(-0.1);
                }
                else {
                    motorLinearSlide.setPower(-gamepad2.left_stick_y);
                }
                
                if (motorLinearSlide.getPower() == 0.0) {
                    motorLinearSlide.setPower(0.05);
                }
            
            // Updating new info for telemetry and sending it to driver station
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                
                telemetry.addData("Color Sensor", "Red: " + colorSensor.red());
                telemetry.addData("Color Sensor", "Green: " + colorSensor.green());
                telemetry.addData("Color Sensor", "Blue: " + colorSensor.blue());
                
                telemetry.addData("Linear Slide Encoder", "Count " + motorLinearSlide.getCurrentPosition());
                
                telemetry.update();
        }
    }
}