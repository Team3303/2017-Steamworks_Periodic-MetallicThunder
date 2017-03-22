#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <math.h>
#include "wpilib.h"
#include <sstream>

class Robot: public frc::IterativeRobot {
public:
	std::shared_ptr<NetworkTable> networkTable;

	Robot() {

		networkTable = NetworkTable::GetTable("GRIP/myContoursReport");

		myRobot.SetExpiration(0.1);
		timer.Start();

		CameraServer::GetInstance()->StartAutomaticCapture(0);
//		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
//		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Blur", 640, 480);

		CameraServer::GetInstance()->AddAxisCamera("10.33.3.19");

		encoder.SetMaxPeriod(.1);
		encoder.SetMinRate(10);
		encoder.SetDistancePerPulse(0.05212915); //should probably be 6 [wheel diameter] * pi / 360 = 0.052359...
		encoder.SetSamplesToAverage(7);
	}

private:

	frc::RobotDrive myRobot { 0, 1, 2, 3 };  // Robot drive system
	frc::Joystick controller { 0 }, joystick_R { 1 }, joystick_L { 2 };  // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Timer regulatorTimer;
	frc::Talon climber1{ 4 }, climber2{ 5 };
	frc::DoubleSolenoid piston{ 0, 1 };
	frc::Compressor* compressor = new Compressor( 0 );
	frc::AnalogGyro gyro{ 1 };
	frc::Encoder encoder{ 0, 1, false, Encoder::EncodingType::k4X };
	
	frc::Talon wrist{ 6 };
	frc::Talon grabbers{ 7 };

	bool isShooting = false;
	bool wasRbPressed = false;
	bool isRbPressed = false;
	bool isIntaking = false;
	bool wasLbPressed = false;
	bool isLbPressed = false;
	bool isClimbing = false;
	bool wasAPressed = false;
	bool isAPressed = false;
	bool isPistonOut = false;
	bool wasBPressed = false;
	bool isBPressed = false;
	bool isRegOn = false;
	bool isXPressed = false;
	bool wasXPressed = false;
	bool isCompressing = false;
	int xAvg = 0, yAvg = 0;
	unsigned int i = 0;

	//turn scaling
	double scale = 1.0;
	double avg = 0;
	double diffL = 0;
	double diffR = 0;

	// Camera
	double centerPixelX = 400.0;  // The center X coord in the Axis camera image
	double centerPixelY = 300.0;  // The center Y coord in the Axis camera image
	double FOV = 67;  //degrees
	double focalLength = centerPixelX / tan( (FOV / 2.0)*(3.14159265358979323846264338327950288419716939937510582 / 180.0) );
	
	bool d_pad_up() {
		if ( (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45) || controller.GetPOV(0) == 315 ) {
			return true;
		} else {
			return false;
		}
	}

	bool d_pad_down() {
		if ( controller.GetPOV(0) >= 135 && controller.GetPOV(0) <= 215 ) {
			return true;
		} else {
			return false;
		}
	}

	bool A(){ return controller.GetRawButton(1); }
	bool B(){ return controller.GetRawButton(2); }
	bool X(){ return controller.GetRawButton(3); }
	bool Y(){ return controller.GetRawButton(4); }
	bool Lb(){ return controller.GetRawButton(5); }  // current gyro loop break
	bool Rb(){ return controller.GetRawButton(6); }
	bool start(){ return controller.GetRawButton(7); } //find actual number
	
	void TargetHook() {
		
		// Calculate angle offset
		// Get networkTables
		std::vector<double> centerXArr = networkTable->GetNumberArray("centerX", llvm::ArrayRef<double>());
		std::vector<double> centerYArr = networkTable->GetNumberArray("centerY", llvm::ArrayRef<double>());
		// Center of all contour Xs
		for (i = 0; i < centerXArr.size(); i++) { xAvg += centerXArr[i]; }
		xAvg /= xAvg;
		// Center of all contour Ys
		for (i = 0; i < centerYArr.size(); i++) { yAvg += centerYArr[i]; }
		xAvg /= centerXArr.size();
		yAvg /= centerYArr.size();

//		double centerX = (centerXArr[0] + centerXArr[1]) / 2;
		double pixelOffset = xAvg - centerPixelX;
		double angleOffset = atan(pixelOffset / focalLength) * (180 / 3.14159);  // degrees
		
	 /* gyro.Reset();
		while( !(gyro.GetAngle() > (angleOffset - 1) && gyro.GetAngle() < (angleOffset + 1)) && !controller.GetRawButton(button) ) 
		{
			double angleRemaining = angleOffset - gyro.GetAngle();
			double turnSpeed =  angleRemaining / (FOV / 2.0);
			myRobot.TankDrive(turnSpeed, -turnSpeed);
		}
		
		myRobot.Drive(0.0, 0.0); */
		
		Align(angleOffset, FOV / 2.0);
		
	}
	
	// Rotate robot to the right by angle
	void Align(double angle, double scale){   //takes angle in degrees, and scale in max degrees
		gyro.Reset();
		while( !(gyro.GetAngle() > (angle - 1) && gyro.GetAngle() < (angle + 1)) && !Lb() ) 
		{
			double angleRemaining = angle - gyro.GetAngle();
			double turnSpeed =  angleRemaining / scale + 0.1;
			myRobot.TankDrive(turnSpeed, -turnSpeed);
		}
		
		myRobot.Drive(0.0, 0.0);
	}
	
	// TODO:Distance Tracking
	void ForwardDistance(double dist){
		encoder.Reset();
		double distLeft = dist;

		int counter = 0;
		while(encoder.GetDistance() < dist && !Lb()) {
			std::stringstream steam;
			std::string encoderValue;
			steam << encoder.GetDistance();
			steam >> encoderValue;
			SmartDashboard::PutString("DB/String 1", encoderValue);


			std::stringstream steam2;
			std::string counterValue;
			steam2 << counter;
			steam2 >> counterValue;
			SmartDashboard::PutString("DB/String 2", counterValue);

			counter++;
			distLeft = dist - encoder.GetDistance();
			myRobot.Drive(/*distLeft < 24 ? distLeft / 96 + 0.1: 00.25*/ 0.15, 0.0);
		}

		myRobot.Drive(0.0, 0.0);
	}

	// XXX:Autonomous
	void AutonomousInit() override {
		gyro.InitGyro();
		hasDriven = false;
		ForwardDistance( (9 * 12) - 32 );
		timer.Reset();
		timer.Start();
	}

	bool hasDriven = false;
	void AutonomousPeriodic() override {
		
		std::stringstream stream;
		std::string gyroValue;
		stream << gyro.GetAngle();
		stream >> gyroValue;
		SmartDashboard::PutString("DB/String 0", gyroValue);

		std::stringstream steam;
		std::string encoderValue;
		steam << encoder.GetDistance();
		steam >> encoderValue;
		SmartDashboard::PutString("DB/String 1", encoderValue);

		if (timer.Get() < 0.5)
			grabbers.Set( 1 );
		else
			grabbers.Set( 0 );

//		double driveDist = SmartDashboard::GetNumber("DB/Slider 2", 2.0);
//
//
//		if(!hasDriven){
//			ForwardDistance(driveDist);
//		}
		
		hasDriven = true;
		// Drive for driveTime seconds
//		if (timer.Get() < driveTime) {
//			myRobot.Drive(1.0, 0.0);  // Drive forwards half speed
//		} else if (timer.Get() < (driveTime + 0.5)){
//			myRobot.Drive(0.0, 0.0);  // Stop robot
//		} else if(SmartDashboard::GetBoolean("DB/Button 0", false)){
//			if (timer.Get() < (driveTime + 0.5 + 0.5)){
//				piston.Set(DoubleSolenoid::Value::kReverse);            //fire piston
//			} else if (timer.Get() < (driveTime + 0.5 + 0.5 + 0.5)){
//				piston.Set(DoubleSolenoid::Value::kReverse);            //retract piston
//			} else if (timer.Get() < (driveTime + 0.5 + 0.5 + 0.5 + 0.5)){
//				myRobot.Drive(-0.5, 0.0);
//			}
//		}else{
//			myRobot.Drive(0.0, 0.0);
//		}
	}

	void TeleopInit() override {
		timer.Stop();
		timer.Reset();
		regulatorTimer.Stop();
		regulatorTimer.Reset();
		gyro.InitGyro();
	}

	void TeleopPeriodic() override {
		/*
		 * FIXME: A Network of Tables
		 */

		std::vector<double> arr = networkTable->GetNumberArray("area", llvm::ArrayRef<double>());
//		for(unsigned int i = 0; i < arr.size(); i++){
//			std::cout << arr[i] << std::endl;
//		}

		//gyro to dashboard
		std::stringstream stream;
		std::string gyroValue;
		stream << gyro.GetAngle();
		stream >> gyroValue;
		SmartDashboard::PutString("DB/String 0", gyroValue);

		std::stringstream steam;
		std::string encoderValue;
		steam << encoder.GetDistance();
		steam >> encoderValue;
		SmartDashboard::PutString("DB/String 1", encoderValue);

		// TODO: Make Omnidrive WCoast
		scale = SmartDashboard::GetNumber("DB/Slider 3", 1.0);
		avg = (-joystick_L.GetY() - joystick_R.GetY()) / 2.0;
//		diffL = -joystick_L.GetY() - avg;
//		diffR = -joystick_R.GetY() - avg;

		if(joystick_R.GetRawButton(2)){
			myRobot.TankDrive(joystick_R.GetY(),joystick_L.GetY());
			//myRobot.TankDrive(-(avg + diffR * scale), -(avg + diffL * scale));
//			omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
//			omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		} else if(joystick_R.GetRawButton(3)){
			myRobot.TankDrive(-joystick_L.GetY() * scale,-joystick_R.GetY() * scale);
//			omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
//			omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		}else{
			myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
			//myRobot.TankDrive(avg + diffL * scale, avg + diffR * scale);
//			omniwheels1.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
//			omniwheels2.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
		}

		// Climber Controls
		double hangSpeed = SmartDashboard::GetNumber( "DB/Slider 2", 0 );
		double grabSpeed = SmartDashboard::GetNumber( "DB/Slider 0", 0 );
		double climbSpeed = SmartDashboard::GetNumber( "DB/Slider 1", 0 );

		// Climber reverse and Hold
		if(A()) {
			climber1.Set( climbSpeed );
			climber2.Set( climbSpeed );
		} else if(Y()) {
			climber1.Set( -grabSpeed );
			climber2.Set( -grabSpeed );
		} else if (d_pad_up()) {
			climber1.Set( hangSpeed );
			climber2.Set( hangSpeed );
		} else if (d_pad_down()){
			climber1.Set( grabSpeed );
			climber2.Set( grabSpeed );
		} else {
			climber1.Set( 0.0 );
			climber2.Set( 0.0 );
		}
		
		// Gyro and vision testing

		if(X()){
			Align(45.0, 90.0);
			//TargetHook();
		}

		if( B() )
			ForwardDistance( (9 * 12) - 32 );

		if(start()){
			encoder.Reset();
			gyro.Reset();
		}

		// FalconZ Super Lifter and Lowerer Mode
		if (controller.GetRawAxis(2) > 0)
			wrist.Set(-0.75);
		else if (controller.GetRawAxis(3) > 0)
			wrist.Set( 0.75);
		else wrist.Set(0);

		// Gear In
		if ( Lb() )
			grabbers.Set(-1);
		// Gear Out
		else if ( Rb() )
			grabbers.Set(1);
		else grabbers.Set(0);
	}

	void TestPeriodic() override { lw->Run(); }
};

START_ROBOT_CLASS(Robot);
