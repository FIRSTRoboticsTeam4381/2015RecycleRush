//Mecanum
//Rec Rush 2015

#include "WPILib.h"

/*
 * FRC 2015 Team 4381 C++
 * Sam Maddox
 */

class FakeLimitSwitch {
public:
	FakeLimitSwitch(int channel) {
	}

	bool Get() {
		Wait(0.5);
		return true;
	}

};

class AutonomousIndex {
	int index;
public:
	AutonomousIndex(int i) {
		index = i;
	}
	int getIndex() {
		return index;
	}

};

class Robot: public IterativeRobot {

	const double VOLTAGE_TO_PICK = 0.05;

	const double CURRENT_WHEN_ARM_STOPPED = 1;

	DriverStation *ds;

	// Channels for the wheels
	const static int frontLeftChannel = 0;
	const static int rearLeftChannel = 1;
	const static int frontRightChannel = 3;
	const static int rearRightChannel = 2;

	const double VOLTAGE_TOLERANCE = 0.07;

	const double ANGLE_TOLERANCE = 0.005;

	RobotDrive robotDrive;
	Joystick stick;
	const static int STICK_CHANNEL = 0;

	Joystick liftStick;
	const static int LIFTSTICK_CHANNEL = 1;

	Talon chainLift;
	const static int CHAINLIFT_PWM = 4;

	DigitalInput maxUp;
	const static int MAXUP_DIO = 1;//<TODO fix>

	FakeLimitSwitch maxDown;
	const static int MAXDOWN_DIO = 0; 		//<TODO fix>

	DigitalInput midPoint;
	const static int MIDPOINT_DIO = 3;

	DigitalInput autoSwitch1;
	const static int AUTOSWITCH1_DIO = 2;

	DigitalInput autoSwitch2;
	const static int AUTOSWITCH2_DIO = 4;


	BuiltInAccelerometer accel;


	SendableChooser *autoMode;


	AnalogInput leftIR;
	const static int LEFTIR_LOC = 1;

	AnalogInput rightIR;
	const static int RIGHTIR_LOC = 3;


	double leftIRZero, rightIRZero;


	int tick;


	Talon canGrabber;
	const static int CANGRAB_PWM = 9;
	//Pdp 14 and 15 for cangrabber


	PowerDistributionPanel pdp;
public:
	Robot() :
			robotDrive(new Talon(frontLeftChannel), new Talon(rearLeftChannel),
					new Talon(frontRightChannel), new Talon(rearRightChannel)), stick(
					STICK_CHANNEL), liftStick(LIFTSTICK_CHANNEL), chainLift(
					CHAINLIFT_PWM), maxUp(MAXUP_DIO), maxDown(MAXDOWN_DIO), midPoint(
					MIDPOINT_DIO), autoSwitch1(AUTOSWITCH1_DIO), autoSwitch2(
					AUTOSWITCH2_DIO), leftIR(LEFTIR_LOC), rightIR(RIGHTIR_LOC), canGrabber(
					CANGRAB_PWM) {
		SmartDashboard::init();
		ds = DriverStation::GetInstance();
		SmartDashboard::PutString("STATUS:", "INITIALIZING");
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		autoMode = new SendableChooser();
		autoMode->AddDefault("00: Do Nothing", new AutonomousIndex(0));
		autoMode->AddObject("01: Just Drive Forward", new AutonomousIndex(1));
		autoMode->AddObject("02: Stack bin on tote, turn 90, go",
				new AutonomousIndex(2));
		autoMode->AddObject("03: Lift up and go forward",
				new AutonomousIndex(3));
		autoMode->AddObject("04: Lift up, turn 90, go", new AutonomousIndex(4));
		autoMode->AddObject("05: Stack 3 bins w/ correction constant",
				new AutonomousIndex(5));
		autoMode->AddObject("06: Stack 3 bins w/ acclerometer",
				new AutonomousIndex(6));
		autoMode->AddObject("07: Grab first bin from landfill",
				new AutonomousIndex(7));
		autoMode->AddObject("08: Grab yellow bin and back up TO RAMP",
				new AutonomousIndex(8));
		autoMode->AddObject("09: Grab two bins from landfill, slide sideways",
				new AutonomousIndex(9));
		autoMode->AddObject("10: Grab from landfill, over by turning",
				new AutonomousIndex(10));
		autoMode->AddObject("11: Grab yellow bin, back up to AUTOZONE, drop",
				new AutonomousIndex(11));
		autoMode->AddObject("12: Grab yellow bin, back up to AUTOZONE, stop",
				new AutonomousIndex(12));
		autoMode->AddObject("13: Steal two green cans. Gottta go fast",
				new AutonomousIndex(13));
		autoMode->AddObject("99: CUPID SHUFFLE", new AutonomousIndex(99));
		SmartDashboard::PutString("Both Off", "Pick from radio list");
		SmartDashboard::PutString("On-Off", "Grab yellow and back up");
		SmartDashboard::PutString("Off-On",
				"Grab, turn right, drive, turn left");
		SmartDashboard::PutString("Both On",
				"Lift up, turn 90, drive to auto zone");
		SmartDashboard::PutData("BOTH SWITCHES ON: Pick One:", autoMode);
		SmartDashboard::PutData(Scheduler::GetInstance());
		SmartDashboard::PutString("STATUS:", "READY");

		SmartDashboard::PutBoolean("Smart Dashboard Enabled", false);

		tick = 0;

		leftIRZero = 0;
		rightIRZero = 0;
	}

	void zeroSanics() {
		leftIRZero = leftIR.GetAverageVoltage();
		rightIRZero = rightIR.GetAverageVoltage();
	}

//---------------------------- TELEOP ---------------- MECANUM WITH JOYSTICKS AND CHAINLIFT ---------------
	void TeleopInit() {
		robotDrive.SetSafetyEnabled(false);
		chainLift.SetSafetyEnabled(false);
		tick = 0;
		SmartDashboard::PutString("STATUS:", "TELEOPERATED");
	}

	void TeleopPeriodic() {

		if(tick==10) if (ds->IsSysBrownedOut()) {
			ds->ReportError("[ERROR] BROWNOUT DETECTED!!");
		}
		if(tick == 15) if (!ds->IsNewControlData()) {
			ds->ReportError(
					"[ERROR] NO DATA FROM DRIVER STATION IN THIS TICK!");
		}
		if(tick==20) if (!ds->IsDSAttached()) {
			ds->ReportError("[ERROR] DRIVER STATION NOT DETECTED!");
		}

		if (stick.GetRawButton(10))
			zeroSanics();

		if (stick.GetRawButton(8)) {
			leftIRZero = 0;
			rightIRZero = 0;
		}
		tick++;

		if (liftStick.GetRawButton(2)) {
			double canScale = liftStick.GetRawAxis(2);
			canScale += 1;
			canScale = 2 - canScale;
			canScale /= 2;
			canGrabber.SetSpeed(canScale);
		} else if (liftStick.GetRawButton(3)) {
			double canScale = liftStick.GetRawAxis(2);
			canScale += 1;
			canScale = 2 - canScale;
			canScale /= 2;
			canGrabber.SetSpeed(-canScale);
		} else
			canGrabber.SetSpeed(0);

		double speed;

		//Calculate scalar to use for POV/Adjusted drive
		double scale = stick.GetRawAxis(3);
		scale += 1;
		scale = 2 - scale;
		scale /= 2;
		//Use pov/hat switch for movement if enabled
		if (stick.GetRawButton(1) && stick.GetRawButton(2)) {
			AutomaticLineup();
		} else if (stick.GetRawButton(1)) {
			double leftVolts = leftIR.GetAverageVoltage() - leftIRZero;
			double rightVolts = rightIR.GetAverageVoltage() - leftIRZero;

			if (rightVolts + VOLTAGE_TOLERANCE > leftVolts
					&& rightVolts - VOLTAGE_TOLERANCE < leftVolts) {
				robotDrive.MecanumDrive_Cartesian(0, 0, 0);
			} else if (rightVolts > leftVolts)
				robotDrive.MecanumDrive_Cartesian(0, 0, 0.2);
			else if (leftVolts > rightVolts)
				robotDrive.MecanumDrive_Cartesian(0, 0, -0.2);
		} else if (stick.GetRawButton(6)) {
			//Rotate
			robotDrive.MecanumDrive_Polar(0, 0, scale);
		} else if (stick.GetRawButton(5)) {
			//Rotate
			robotDrive.MecanumDrive_Polar(0, 0, -scale);
		} else if (stick.GetPOV(0) != -1) {
			//If POV moved, move polar (getPOV returns an angle in degrees)
			robotDrive.MecanumDrive_Polar(scale, -stick.GetPOV(0), 0);
		} else if (stick.GetRawButton(2)) {
			//Drive with scalar
			robotDrive.MecanumDrive_Cartesian(-stick.GetRawAxis(0) * scale,
					stick.GetRawAxis(1) * scale, stick.GetRawAxis(2) * scale);
		} else {
			//Drive normally
			robotDrive.MecanumDrive_Cartesian(-stick.GetX(), stick.GetY(),
					stick.GetZ());
		}
		speed = -liftStick.GetY();

		//bool canGoUp = maxUp.Get();
		bool canGoUp = true;
		//bool canGoDown = maxDown.Get();
		bool canGoDown = true;

		//If at a limit switch and moving in that direction, stop
		if (speed > 0 && !canGoUp)
			speed = 0;
		if (speed < 0 && !canGoDown)
			speed = 0;

		chainLift.SetSpeed(speed);

		if (tick >50) {
			if (SmartDashboard::GetBoolean("Smart Dashboard Enabled")) {
				//Smart Dash outputs
				//SmartDashboard::PutNumber("X Acceleration: ", accel.GetX());
				//SmartDashboard::PutNumber("Y Acceleration: ", accel.GetY());
				//SmartDashboard::PutNumber("Z Acceleration: ", accel.GetZ());
				SmartDashboard::PutBoolean("Switch 1: (up)", maxUp.Get());
				SmartDashboard::PutBoolean("Switch 2: (down)", maxDown.Get());
				SmartDashboard::PutBoolean("Switch 3: (mid)", midPoint.Get());
				SmartDashboard::PutBoolean("Auto switch A: ",
						autoSwitch1.Get());
				SmartDashboard::PutBoolean("Auto switch B: ",
						autoSwitch2.Get());

				//SmartDashboard::PutBoolean("RobotDrive Alive?",
					//	robotDrive.IsAlive());
				//SmartDashboard::PutBoolean("ChainLift Alive?",
					//	robotDrive.IsAlive());

				SmartDashboard::PutNumber("Left Sensor",
						leftIR.GetAverageVoltage());
				SmartDashboard::PutNumber("Right Sensor",
						rightIR.GetAverageVoltage());

				SmartDashboard::PutNumber("Left w zero",
						leftIR.GetAverageVoltage() - leftIRZero);
				SmartDashboard::PutNumber("Rigt w zero",
						rightIR.GetAverageVoltage() - rightIRZero);

				SmartDashboard::PutNumber("PDP 14 Current", pdp.GetCurrent(14));
				SmartDashboard::PutNumber("PDP 15 Current", pdp.GetCurrent(15));
			}

			tick = 0;
		}
	}

	void Disabled() {
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0);
		SmartDashboard::PutString("STATUS:", "DISABLED");
	}
	/*
	 * ---------------------------------	AUTONOMOUS STUFF	---------------------------------
	 */
	//Choose which auto to use
	void AutonomousInit() {

		chainLift.SetSpeed(0);
		canGrabber.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "STARTING AUTO");
		robotDrive.SetSafetyEnabled(false);
		chainLift.SetSafetyEnabled(false);

		SmartDashboard::PutBoolean("Auto switch A: ", autoSwitch1.Get());
		SmartDashboard::PutBoolean("Auto switch B: ", autoSwitch2.Get());

		//Select auto type
		if (autoSwitch1.Get()) {
			if (autoSwitch2.Get())
				AutonomousType4();
			else
				//1 on 2 grab n back
				AutonomousType8();
		} else {
			if (autoSwitch2.Get())
				//1 off, 2 on: grab n turn
				AutonomousType10();
			else {
				SmartAutoPicker();
			}
			//Do Nothing
		}
	}

	void SmartAutoPicker() {
		AutonomousIndex* indexed = (AutonomousIndex*) autoMode->GetSelected();
		int index = indexed->getIndex();
		switch (index) {
		case 0: {
			SmartDashboard::PutString("STATUS:", "NOT PERFORMING ANY AUTO");
			break;
		}
		case 1: {
			AutonomousType1();
			break;
		}
		case 2: {
			AutonomousType2();
			break;
		}
		case 3: {
			AutonomousType3();
			break;
		}
		case 4: {
			AutonomousType4();
			break;
		}
		case 5: {
			AutonomousType5();
			break;
		}
		case 6: {
			AutonomousType6();
			break;
		}
		case 7: {
			AutonomousType7();
			break;
		}
		case 8: {
			AutonomousType8();
			break;
		}
		case 9: {
			AutonomousType9();
			break;
		}
		case 10: {
			AutonomousType10();
			break;
		}
		case 11: {
			AutonomousType11();
			break;
		}
		case 12: {
			AutonomousType12();
			break;
		}
		case 13: {
			AutonomousType13();
			break;
		}

		case 99: {
			CupidShuffle();
			break;
		}
		}
	}

	void AutonomousType1() {			//Just drive forward
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 1");
		//<TODO>-0.2 y for 3 seconds =43 inches --> 1 second at full speed is 71.66667 inches
		robotDrive.MecanumDrive_Cartesian(0, -0.36, 0);
		if (WaitF(1.75))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 1 COMPLETE");
	}

	void AutonomousType2() {		//Pick tote and bin, move to auto zone
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 2");
		chainLift.SetSpeed(0.5);
		while (midPoint.Get() && maxUp.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Polar(0.3, 0, 0);
		if (WaitF(1.6))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		chainLift.SetSpeed(-0.2);
		if (WaitF(0.8))
			return;
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Polar(-0.3, 0, 0);
		if (WaitF(1.6))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		chainLift.SetSpeed(-0.3);
		while (maxDown.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Polar(0.2, 0, 0);
		if (WaitF(2))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		chainLift.SetSpeed(0.4);
		while (midPoint.Get() && maxUp.Get()) {
		}
		chainLift.SetSpeed(0);
		//turn 90 deg
		robotDrive.MecanumDrive_Polar(0, 0, -0.3);
		if (WaitF(4))
			return;
		robotDrive.MecanumDrive_Polar(0.5, 0, 0);
		if (WaitF(2.5))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		chainLift.SetSpeed(-0.4);
		while (maxDown.Get() && IsAutonomous()) {
		}
		chainLift.SetSpeed(0);
		SmartDashboard::PutString("STATUS:", "AUTO 2 COMPLETE");

	}

	void AutonomousType3() {		//Grab a bin/trash bin, and move forward
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 3");
		chainLift.SetSpeed(0.5);
		while (midPoint.Get() && maxUp.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, -0.75, 0);
		if (WaitF(1.75))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.5);
		while (maxDown.Get()) {
		}
		chainLift.SetSpeed(0);
		SmartDashboard::PutString("STATUS:", "AUTO 3 COMPLETE");
	}

	void AutonomousType4() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 4");

		//Lift, turn, drive
		chainLift.SetSpeed(0.5);
		while (midPoint.Get() && maxUp.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Polar(0, 0, 0.3);

		if (WaitF(2.5))
			return;
		robotDrive.MecanumDrive_Polar(0.25, 0, 0);
		if (WaitF(5.6))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0.3);
		if (WaitF(2))
			return;
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		//chainLift.SetSpeed(-0.4);
		//while (maxDown.Get() && IsAutonomous()) {
		//}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 4 COMPLETE");
	}

	void AutonomousType5() { //All 3 totes with correction constant
		double CORRECTION = -0.021;
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 5");
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);

		//Move forward
		robotDrive.MecanumDrive_Cartesian(0, -0.5, CORRECTION);
		Wait(1.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//Drop
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.2);
		Wait(0.35);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//back a little to unhook from stack
		robotDrive.MecanumDrive_Cartesian(0, 0.2, 0);
		Wait(0.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//down to grab stack
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.8);
		while (maxDown.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}

		//forward a little
		robotDrive.MecanumDrive_Cartesian(0, -0.3, 0);
		Wait(0.4);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//pick up stack
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);

		//forward
		robotDrive.MecanumDrive_Cartesian(0, -0.5, CORRECTION);
		Wait(1.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//down
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.2);
		Wait(0.35);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//back
		robotDrive.MecanumDrive_Cartesian(0, 0.2, 0);
		Wait(0.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//down
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.8);
		while (maxDown.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		//forward
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, -0.3, CORRECTION);
		Wait(0.4);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//up
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);
		//sideways to zone
		robotDrive.MecanumDrive_Cartesian(0.5, 0, 0);
		if (WaitF(0.5))
			return;
		//stop
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 5 COMPLETE");
	}

	void AutonomousType6() { //All 3 totes with accelerometer

		//Pick up 3 bins using gyroscope to correct corse
		Wait(1);
		SmartDashboard::PutString("STATUS:", "AUTO 6 (ACCEL)");
		//Move forward
		//robotDrive.MecanumDrive_Cartesian(0, -0.5, 0);
		//Wait(0.1);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//Lift up first box
		//robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);
		for (int j = 0; j < 1400; j++) {
			if (!IsAutonomous() || !IsEnabled())
				return;
			//Move forward
			//CORRECT WITH ACCEL
			robotDrive.MecanumDrive_Cartesian(-accel.GetX(), 0, 0);
			Wait(0.001);
		}

		//Drop
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.3);
		Wait(1);

		if (!IsAutonomous() || !IsEnabled())
			return;

		//back a little to unhook from stack
		robotDrive.MecanumDrive_Cartesian(0, 0.5, 0);
		Wait(0.3);

		if (!IsAutonomous() || !IsEnabled())
			return;

		//down to grab stack
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.8);
		while (maxDown.Get()) {
		}

		if (!IsAutonomous() || !IsEnabled())
			return;

		//forward a little
		robotDrive.MecanumDrive_Cartesian(0, -0.4, 0);
		Wait(1.2);

		if (!IsAutonomous() || !IsEnabled())
			return;

		//pick up stack
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);

		for (int j = 0; j < 1500; j++) {
			if (!IsAutonomous() || !IsEnabled())
				return;
			//Move forward
			robotDrive.MecanumDrive_Cartesian(-accel.GetX(), 0, 0);
			Wait(0.001);
		}

		//down
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.3);
		Wait(0.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//back
		robotDrive.MecanumDrive_Cartesian(0, 0.7, 0);
		Wait(0.5);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//down
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.8);
		while (maxDown.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		//forward
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, -0.3, 0);
		Wait(2);
		if (!IsAutonomous() || !IsEnabled())
			return;
		//up
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.8);
		while (midPoint.Get() && maxUp.Get()) {
			if (!IsAutonomous() || !IsEnabled())
				return;
		}
		chainLift.SetSpeed(0);
		//turn 90 deg
		robotDrive.MecanumDrive_Polar(0, 0, -0.3);
		Wait(4);
		robotDrive.MecanumDrive_Polar(0.5, 0, 0);
		Wait(2.5);
		robotDrive.MecanumDrive_Polar(0, 0, 0);
		chainLift.SetSpeed(-0.4);
		while (maxDown.Get() && IsAutonomous()) {
		}
		chainLift.SetSpeed(0);
		SmartDashboard::PutString("STATUS:", "AUTO 6 COMPLETE");
	}
	//
	void AutonomousType7() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 7");
		robotDrive.MecanumDrive_Cartesian(0, -0.2, 0);
		if (WaitF(1.2))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.2, 0);
		if (WaitF(1.6))
			return;
		SmartDashboard::PutString("STATUS:", "AUTO 7 COMPLETE");
	}

	//Grab first yellow, back up to ramp
	void AutonomousType8() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 8");
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && IsEnabled() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.4, 0);
		if (WaitF(3.8))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 8 COMPLETE");
	}

	//Grab first two, strafe right
	void AutonomousType9() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 9");
		robotDrive.MecanumDrive_Cartesian(0, -0.2, 0);
		if (WaitF(1.2))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.2, 0);
		if (WaitF(1.6))
			return;
		robotDrive.MecanumDrive_Cartesian(-0.8, 0, 0);
		if (WaitF(5))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 9 COMPLETE");
	}

	//Grab first two and turn to go right
	void AutonomousType10() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 10");
		robotDrive.MecanumDrive_Cartesian(0, -0.2, 0);
		if (WaitF(1.2))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.4, 0);
		if (WaitF(1.6))
			return;

		robotDrive.MecanumDrive_Polar(0, 0, 0.3);
		if (WaitF(2.6))
			return;

		robotDrive.MecanumDrive_Cartesian(0, -0.4, 0);
		if (WaitF(1))
			return;

		robotDrive.MecanumDrive_Polar(0, 0, -0.3);
		if (WaitF(2.6))
			return;

		robotDrive.MecanumDrive_Cartesian(0, -0.4, 0);
		if (WaitF(1.6))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 10 COMPLETE");
	}

	//Grab first yellow, back up to auto zone, drop
	void AutonomousType11() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 11");
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && IsEnabled() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.4, 0);
		if (WaitF(3))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		chainLift.SetSpeed(-0.5);
		while (IsAutonomous() && IsEnabled() && maxDown.Get()) {
		}
		chainLift.SetSpeed(0);
		SmartDashboard::PutString("STATUS:", "AUTO 11 COMPLETE");
	}

	//Grab first yellow, back up to auto zone, DON'T DROP
	void AutonomousType12() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 12");
		chainLift.SetSpeed(0.5);
		while (IsAutonomous() && IsEnabled() && maxUp.Get() && midPoint.Get()) {
		}
		chainLift.SetSpeed(0);
		robotDrive.MecanumDrive_Cartesian(0, 0.4, 0);
		if (WaitF(3))
			return;
		robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 12 COMPLETE");
	}

	//Steal cans
	void AutonomousType13() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 13");
		robotDrive.MecanumDrive_Cartesian(0, 0.2, 0);
				if (WaitF(1.2))
					return;
				robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		canGrabber.SetSpeed(1);
		if (WaitF(4))
			return;
		canGrabber.SetSpeed(0);
		LinearAcceleration(1, 0, 1, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 13 COMPLETE");
	}

/*	//Steal cans, stop w/ current
	void AutonomousType14() {
		SmartDashboard::PutString("STATUS:", "STARTING AUTO 14");
		canGrabber.SetSpeed(1);
		while (IsAutonomous() && IsEnabled()
				&& pdp.GetCurrent(14) < CURRENT_WHEN_ARM_STOPPED) {
		}
		canGrabber.SetSpeed(0);
		LinearAcceleration(1, 0, 1, 0);
		SmartDashboard::PutString("STATUS:", "AUTO 14 COMPLETE");
	}*/

	void CupidShuffle() {
		SmartDashboard::PutString("STATUS:", "TIME 2 GET DOWWWWWWN");
		//Tempo of song
		static double tempo = 0.41666666667;

		//Repeat # of times
		for (int j = 0; j < 10 && IsAutonomous() && IsEnabled(); j++) {
			//to the left to the left to the left to the left
			for (int k = 0; k < 4; k++) {
				robotDrive.MecanumDrive_Cartesian(-0.2, 0, 0);
				Wait(tempo);
				robotDrive.MecanumDrive_Cartesian(0, 0, 0);
				Wait(tempo);
			}

			//to the right to the right to the right
			for (int k = 0; k < 4; k++) {
				robotDrive.MecanumDrive_Cartesian(0.2, 0, 0);
				Wait(tempo);
				robotDrive.MecanumDrive_Cartesian(0, 0, 0);
				Wait(tempo);
			}

			//kick kick kick kick
			for (int k = 0; k < 4; k++) {
				chainLift.SetSpeed(0.3);
				Wait(tempo);
				chainLift.SetSpeed(-0.3);
				Wait(tempo);
			}
			chainLift.SetSpeed(0);

			//walk it by uself (turn 90)
			robotDrive.MecanumDrive_Polar(0, 0, 0.3);
			Wait(tempo * 8);
		}
		SmartDashboard::PutString("STATUS:", "GIT GUD");

	}

	bool WaitF(double time) {
		double completed = 0;
		while (completed < time) {
			completed += 0.05;
			Wait(0.05);
			if (!IsAutonomous() || !IsEnabled())
				return true;

		}
		return false;
	}

	void AutomaticLineup() {
		double leftVolts = leftIR.GetAverageVoltage() - leftIRZero;
		double rightVolts = rightIR.GetAverageVoltage() - leftIRZero;
		if (leftVolts <= VOLTAGE_TO_PICK && rightVolts <= VOLTAGE_TO_PICK) {
			robotDrive.MecanumDrive_Cartesian(0, 0, 0);
		} else if (leftVolts > VOLTAGE_TO_PICK
				&& rightVolts > VOLTAGE_TO_PICK) {
			robotDrive.MecanumDrive_Cartesian(0, -0.3, 0);
		} else if (leftVolts > VOLTAGE_TO_PICK) {
			robotDrive.MecanumDrive_Cartesian(0, 0, 0.2);
		} else if (rightVolts > VOLTAGE_TO_PICK) {
			robotDrive.MecanumDrive_Cartesian(0, 0, -0.2);
		}
	}

	void LinearAcceleration(double startSpeed, double endSpeed, double time,
			double angle) {
		const double UPDATE_FREQ = 0.05;
		double toChange = endSpeed - startSpeed;
		double tickCount = time / UPDATE_FREQ;
		double speed = startSpeed;
		double tickChange = toChange / tickCount;
		for (int tick = 0; tick < tickCount; tick++) {
			robotDrive.MecanumDrive_Polar(speed, angle, 0);
			speed += tickChange;
		}

	}

};

START_ROBOT_CLASS(Robot);
