%module wpilib
%{
#include <WPILib/WPILib.h>
%}

typedef signed char INT8;
typedef signed short INT16;
typedef signed int INT32;
typedef signed long long INT64;

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned long long UINT64;

/*** ABSTRACT BASES ***/
class ErrorBase;

class Error
{
public:
    typedef tRioStatusCode Code;

	Error();
	~Error();
	Code GetCode() const;
	const char *GetMessage() const;
	const char *GetFilename() const;
	UINT32 GetLineNumber() const;
    const ErrorBase* GetOriginatingObject() const;
	void Clear();
	void Set(Code code, const char* filename, UINT32 lineNumber, const ErrorBase* originatingObject);
};

class ErrorBase
{
public:
	virtual ~ErrorBase();
	virtual Error& GetError();
	virtual const Error& GetError() const;
	virtual void SetError(Error::Code code, const char* filename, UINT32 lineNumber) const;
    virtual void ClearError();
    virtual bool StatusIsFatal() const;
    static Error& GetGlobalError();
};

class SensorBase: public ErrorBase {
public:
	static const UINT32 kSystemClockTicksPerMicrosecond = 40;

	SensorBase();
	virtual ~SensorBase();
	static void SetDefaultAnalogModule(UINT32 slot);
	static void SetDefaultDigitalModule(UINT32 slot);
	static void SetDefaultSolenoidModule(UINT32 slot);
	static void DeleteSingletons();
	static UINT32 GetDefaultAnalogModule() { return m_defaultAnalogModule; }
	static UINT32 GetDefaultDigitalModule() { return m_defaultDigitalModule; }
	static UINT32 GetDefaultSolenoidModule() { return m_defaultSolenoidModule; }
	static bool CheckDigitalModule(UINT32 slot);
	static bool CheckRelayModule(UINT32 slot);
	static bool CheckPWMModule(UINT32 slot);
	static bool CheckSolenoidModule(UINT32 slot);
	static bool CheckAnalogModule(UINT32 slot);
	static bool CheckDigitalChannel(UINT32 channel);
	static bool CheckRelayChannel(UINT32 channel);
	static bool CheckPWMChannel(UINT32 channel);
	static bool CheckAnalogChannel(UINT32 channel);
	static bool CheckSolenoidChannel(UINT32 channel);

	static const UINT32 kDigitalChannels = 14;
	static const UINT32 kAnalogChannels = 8;
	static const UINT32 kAnalogModules = 2;
	static const UINT32 kDigitalModules = 2;
	static const UINT32 kSolenoidChannels = 8;
    static const UINT32 kSolenoidModules = 2;
	static const UINT32 kPwmChannels = 10;
	static const UINT32 kRelayChannels = 8;
	static const UINT32 kChassisSlots = 8;
};

class InterruptableSensorBase : public SensorBase
{
public:
	InterruptableSensorBase();
	virtual ~InterruptableSensorBase();
	virtual void RequestInterrupts(tInterruptHandler handler, void *param) = 0; ///< Asynchronus handler version.
	virtual void RequestInterrupts() = 0;		///< Synchronus Wait version.
	virtual void CancelInterrupts();			///< Free up the underlying chipobject functions.
	virtual void WaitForInterrupt(float timeout); ///< Synchronus version.
	virtual void EnableInterrupts();			///< Enable interrupts - after finishing setup.
	virtual void DisableInterrupts();		///< Disable, but don't deallocate.
	virtual double ReadInterruptTimestamp();		///< Return the timestamp for the interrupt that occurred.
};

class DigitalSource: public InterruptableSensorBase
{
public:
	virtual ~DigitalSource();
	virtual UINT32 GetChannelForRouting() = 0;
	virtual UINT32 GetModuleForRouting() = 0;
	virtual bool GetAnalogTriggerForRouting() = 0;
	virtual void RequestInterrupts(tInterruptHandler handler, void *param) = 0;
	virtual void RequestInterrupts() = 0;
};

class CounterBase
{
public:
	typedef enum {k1X, k2X, k4X} EncodingType;

	virtual ~CounterBase() {}
	virtual void Start() = 0;
	virtual INT32 Get() = 0;
	virtual void Reset() = 0;
	virtual void Stop() = 0;
	virtual double GetPeriod() = 0;
	virtual void SetMaxPeriod(double maxPeriod) = 0;
	virtual bool GetStopped() = 0;
	virtual bool GetDirection() = 0;
};

class SpeedController
{
public:
	virtual ~SpeedController() {};
	virtual void Set(float speed) = 0;
	virtual float Get() = 0;
};

class PWM : public SensorBase
{
public:
	typedef enum {kPeriodMultiplier_1X = 1, kPeriodMultiplier_2X = 2, kPeriodMultiplier_4X = 4} PeriodMultiplier;

	explicit PWM(UINT32 channel);
	PWM(UINT32 slot, UINT32 channel);
	virtual ~PWM();
	void SetRaw(UINT8 value);
	UINT8 GetRaw();
	void SetPeriodMultiplier(PeriodMultiplier mult);
	void EnableDeadbandElimination(bool eliminateDeadband);
	void SetBounds(INT32 max, INT32 deadbandMax, INT32 center, INT32 deadbandMin, INT32 min);
};

class GenericHID
{
public:
	typedef enum {
		kLeftHand = 0,
		kRightHand = 1
	} JoystickHand;

	virtual ~GenericHID() {}

	virtual float GetX(JoystickHand hand = kRightHand) = 0;
	virtual float GetY(JoystickHand hand = kRightHand) = 0;
	virtual float GetZ() = 0;
	virtual float GetTwist() = 0;
	virtual float GetThrottle() = 0;
	virtual float GetRawAxis(UINT32 axis) = 0;

	virtual bool GetTrigger(JoystickHand hand = kRightHand) = 0;
	virtual bool GetTop(JoystickHand hand = kRightHand) = 0;
	virtual bool GetBumper(JoystickHand hand = kRightHand) = 0;
	virtual bool GetRawButton(UINT32 button) = 0;
};

/*** CONCRETE CLASSES ***/

class AnalogChannel : public SensorBase
{
public:
	static const UINT32 kAccumulatorSlot = 1;
	static const UINT32 kAccumulatorNumChannels = 2;
	static const UINT32 kAccumulatorChannels[kAccumulatorNumChannels];

	AnalogChannel(UINT32 slot, UINT32 channel);
	explicit AnalogChannel(UINT32 channel);
	virtual ~AnalogChannel();

	AnalogModule *GetModule();

	INT16 GetValue();
	INT32 GetAverageValue();

	float GetVoltage();
	float GetAverageVoltage();

	UINT32 GetSlot();
	UINT32 GetChannel();

	void SetAverageBits(UINT32 bits);
	UINT32 GetAverageBits();
	void SetOversampleBits(UINT32 bits);
	UINT32 GetOversampleBits();

	UINT32 GetLSBWeight();
	INT32 GetOffset();

	bool IsAccumulatorChannel();
	void InitAccumulator();
	void SetAccumulatorInitialValue(INT64 value);
	void ResetAccumulator();
	void SetAccumulatorCenter(INT32 center);
	void SetAccumulatorDeadband(INT32 deadband);
	INT64 GetAccumulatorValue();
	UINT32 GetAccumulatorCount();
	void GetAccumulatorOutput(INT64 *value, UINT32 *count);
};

class Dashboard : public ErrorBase
{
public:
	enum Type {kI8, kI16, kI32, kU8, kU16, kU32, kFloat, kDouble, kBoolean, kString, kOther};
	enum ComplexType {kArray, kCluster};

	void AddI8(INT8 value);
	void AddI16(INT16 value);
	void AddI32(INT32 value);
	void AddU8(UINT8 value);
	void AddU16(UINT16 value);
	void AddU32(UINT32 value);
	void AddFloat(float value);
	void AddDouble(double value);
	void AddBoolean(bool value);
	void AddString(char* value);
	void AddString(char* value, INT32 length);

	void AddArray(void);
	void FinalizeArray(void);
	void AddCluster(void);
	void FinalizeCluster(void);

	void Printf(const char *writeFmt, ...);

	INT32 Finalize(void);
};

class DigitalInput : public DigitalSource {
public:
	explicit DigitalInput(UINT32 channel);
	DigitalInput(UINT32 slot, UINT32 channel);
	~DigitalInput();
	UINT32 Get();
	UINT32 GetChannel();

	// Digital Source Interface
	virtual UINT32 GetChannelForRouting();
	virtual UINT32 GetModuleForRouting();
	virtual bool GetAnalogTriggerForRouting();
	
	// Interruptable Interface
	virtual void RequestInterrupts(tInterruptHandler handler, void *param=NULL); ///< Asynchronus handler version.
	virtual void RequestInterrupts();		///< Synchronus Wait version.
	void SetUpSourceEdge(bool risingEdge, bool fallingEdge);
};

class DigitalOutput : public SensorBase
{
public:
	explicit DigitalOutput(UINT32 channel);
	DigitalOutput(UINT32 slot, UINT32 channel);
	~DigitalOutput();
	void Set(UINT32 value);
	void Pulse(float length);
	bool IsPulsing();
};

class DriverStation : public SensorBase
{
public:
	enum Alliance {kRed, kBlue, kInvalid};

	virtual ~DriverStation();
	static DriverStation *GetInstance();

	static const UINT32 kBatterySlot = 1;
	static const UINT32 kBatteryChannel = 8;
	static const UINT32 kJoystickPorts = 4;
	static const UINT32 kJoystickAxes = 6;

	float GetStickAxis(UINT32 stick, UINT32 axis);
	short GetStickButtons(UINT32 stick);

	float GetAnalogIn(UINT32 channel);
	bool GetDigitalIn(UINT32 channel);
	void SetDigitalOut(UINT32 channel, bool value);
	bool GetDigitalOut(UINT32 channel);

	bool IsEnabled();
	bool IsDisabled();
	bool IsAutonomous();
	bool IsOperatorControl();
	bool IsNewControlData();
	bool IsFMSAttached();

	UINT32 GetPacketNumber();
	Alliance GetAlliance();
	UINT32 GetLocation();

	float GetBatteryVoltage();

	Dashboard& GetHighPriorityDashboardPacker(void) {return m_dashboardHigh;}
	Dashboard& GetLowPriorityDashboardPacker(void) {return m_dashboardLow;}
	DriverStationEnhancedIO& GetEnhancedIO(void) {return m_enhancedIO;}
};

class DriverStationEnhancedIO : public ErrorBase
{
public:
	enum tDigitalConfig {kUnknown, kInputFloating, kInputPullUp, kInputPullDown, kOutput, kPWM, kAnalogComparator};
	enum tAccelChannel {kAccelX = 0, kAccelY = 1, kAccelZ = 2};
	enum tPWMPeriodChannels {kPWMChannels1and2, kPWMChannels3and4};

	double GetAcceleration(tAccelChannel channel);
	double GetAnalogIn(UINT32 channel);
	double GetAnalogInRatio(UINT32 channel);
	double GetAnalogOut(UINT32 channel);
	void SetAnalogOut(UINT32 channel, double value);
	bool GetButton(UINT32 channel);
	UINT8 GetButtons();
	void SetLED(UINT32 channel, bool value);
	void SetLEDs(UINT8 value);
	bool GetDigital(UINT32 channel);
	UINT16 GetDigitals();
	void SetDigitalOutput(UINT32 channel, bool value);
	tDigitalConfig GetDigitalConfig(UINT32 channel);
	void SetDigitalConfig(UINT32 channel, tDigitalConfig config);
	double GetPWMPeriod(tPWMPeriodChannels channels);
	void SetPWMPeriod(tPWMPeriodChannels channels, double period);
	bool GetFixedDigitalOutput(UINT32 channel);
	void SetFixedDigitalOutput(UINT32 channel, bool value);
	INT16 GetEncoder(UINT32 encoderNumber);
	void ResetEncoder(UINT32 encoderNumber);
	bool GetEncoderIndexEnable(UINT32 encoderNumber);
	void SetEncoderIndexEnable(UINT32 encoderNumber, bool enable);
	double GetTouchSlider();
	double GetPWMOutput(UINT32 channel);
	void SetPWMOutput(UINT32 channel, double value);
	UINT8 GetFirmwareVersion();
};

class DriverStationLCD : public SensorBase
{
public:
	static const UINT32 kSyncTimeout_ms = 20;
	static const UINT16 kFullDisplayTextCommand = 0x9FFF;
	static const INT32 kLineLength = 21;
	static const INT32 kNumLines = 6;
	enum Line {kMain_Line6=0, kUser_Line1=0, kUser_Line2=1, kUser_Line3=2, kUser_Line4=3, kUser_Line5=4, kUser_Line6=5};

	virtual ~DriverStationLCD();
	static DriverStationLCD *GetInstance();

	void UpdateLCD();
 
	void Clear();
    
    %extend
    {
        void Print(Line line, INT32 startingColumn, const char *s)
        {
            $self->Printf(line, startingColumn, "%s", s);
        }
        
        void PrintLine(Line line, const char *s)
        {
            $self->PrintfLine(line, "%s", s);
        }
    }
};

class Encoder: public SensorBase, public CounterBase
{
public:
	Encoder(UINT32 aChannel, UINT32 bChannel, bool reverseDirection=false, EncodingType encodingType = k4X);
	Encoder(UINT32 aSlot, UINT32 aChannel, UINT32 bSlot, UINT32 _bChannel, bool reverseDirection=false, EncodingType encodingType = k4X);
	Encoder(DigitalSource *aSource, DigitalSource *bSource, bool reverseDirection=false, EncodingType encodingType = k4X);
	virtual ~Encoder();

	// CounterBase interface
	void Start();
	INT32 Get();
	INT32 GetRaw();
	void Reset();
	void Stop();
	double GetPeriod();
	void SetMaxPeriod(double maxPeriod);
	bool GetStopped();
	bool GetDirection();
	double GetDistance();
	double GetRate();
	void SetMinRate(double minRate);
	void SetDistancePerPulse(double distancePerPulse);
	void SetReverseDirection(bool reverseDirection);
};

class Gyro : public SensorBase
{
public:
	static const UINT32 kOversampleBits = 10;
	static const UINT32 kAverageBits = 0;
	static const float kSamplesPerSecond = 50.0;
	static const float kCalibrationSampleTime = 5.0;
	static const float kDefaultVoltsPerDegreePerSecond = 0.007;

	Gyro(UINT32 slot, UINT32 channel);
	explicit Gyro(UINT32 channel);
	explicit Gyro(AnalogChannel *channel);
	virtual ~Gyro();
	float GetAngle();
	void SetSensitivity(float voltsPerDegreePerSecond);
	void Reset();
};

class Jaguar : public PWM, public SpeedController
{
public:
	explicit Jaguar(UINT32 channel);
	Jaguar(UINT32 slot, UINT32 channel);
	virtual ~Jaguar();
	float Get();
	void Set(float value);
};

class Joystick : public GenericHID
{
public:
	static const UINT32 kDefaultXAxis = 1;
	static const UINT32 kDefaultYAxis = 2;
	static const UINT32 kDefaultZAxis = 3;
	static const UINT32 kDefaultTwistAxis = 4;
	static const UINT32 kDefaultThrottleAxis = 3;
	typedef enum
	{
		kXAxis, kYAxis, kZAxis, kTwistAxis, kThrottleAxis, kNumAxisTypes
	} AxisType;
	static const UINT32 kDefaultTriggerButton = 1;
	static const UINT32 kDefaultTopButton = 2;
	typedef enum
	{
		kTriggerButton, kTopButton, kNumButtonTypes
	} ButtonType;

	explicit Joystick(UINT32 port);
	Joystick(UINT32 port, UINT32 numAxisTypes, UINT32 numButtonTypes);
	virtual ~Joystick();

	UINT32 GetAxisChannel(AxisType axis);
	void SetAxisChannel(AxisType axis, UINT32 channel); 

	virtual float GetX(JoystickHand hand = kRightHand);
	virtual float GetY(JoystickHand hand = kRightHand);
	virtual float GetZ();
	virtual float GetTwist();
	virtual float GetThrottle();
	virtual float GetAxis(AxisType axis);
	float GetRawAxis(UINT32 axis);

	virtual bool GetTrigger(JoystickHand hand = kRightHand);
	virtual bool GetTop(JoystickHand hand = kRightHand);
	virtual bool GetBumper(JoystickHand hand = kRightHand);
	virtual bool GetButton(ButtonType button);
	bool GetRawButton(UINT32 button);
	static Joystick* GetStickForPort(UINT32 port);
	
	virtual float GetMagnitude();
	virtual float GetDirectionRadians();
	virtual float GetDirectionDegrees();
};

class Relay : public SensorBase {
public:
	typedef enum {kOff, kOn, kForward, kReverse} Value;
	typedef enum {kBothDirections, kForwardOnly, kReverseOnly} Direction;

	Relay(UINT32 slot, UINT32 channel, Direction direction = kBothDirections);
	virtual ~Relay();

	void Set(Value value);
	void SetDirection(Direction direction);
};

class Solenoid : public SensorBase {
public:
	explicit Solenoid(UINT32 channel);
	Solenoid(UINT32 slot, UINT32 channel);
	~Solenoid();
	void Set(bool on);
	bool Get();
	char GetAll();
};

class Timer
{
public:
	Timer();
	virtual ~Timer();
	double Get();
	void Reset();
	void Start();
	void Stop();
	bool HasPeriodPassed(double period);

	static double GetFPGATimestamp(void);
	static double GetPPCTimestamp(void);
};

class Victor : public PWM, public SpeedController
{
public:
	explicit Victor(UINT32 channel);
	Victor(UINT32 slot, UINT32 channel);
	virtual ~Victor();
	void Set(float value);
	float Get();
};

class Watchdog : public SensorBase
{
public:
	static const double kDefaultWatchdogExpiration = 0.5;

	Watchdog();
	virtual ~Watchdog();
	bool Feed();
	void Kill();
	double GetTimer();
	double GetExpiration();
	void SetExpiration(double expiration);
	bool GetEnabled();
	void SetEnabled(bool enabled);
	bool IsAlive();
	bool IsSystemActive();
};

/*** ROBOT RUNTIME INFORMATION ***/

%inline %{

bool isEnabled()
{
    return RobotBase::getInstance().IsEnabled();
}

bool isDisabled()
{
    return RobotBase::getInstance().IsDisabled();
}

bool isAutonomous()
{
    return RobotBase::getInstance().IsAutonomous();
}

bool isOperatorControl()
{
    return RobotBase::getInstance().IsOperatorControl();
}

bool isSystemActive()
{
    return RobotBase::getInstance().IsSystemActive();
}

bool isNewDataAvailable()
{
    return RobotBase::getInstance().IsNewDataAvailable();
}

Watchdog *getWatchdog()
{
    return &RobotBase::getInstance().GetWatchdog();
}

%}
