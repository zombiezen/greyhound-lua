/*
 *  wpilib.i - SWIG bindings for wpilib
 *  Greyhound Lua
 *
 *  Copyright (c) 2010 Ross Light
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 *  and/or sell copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

%module wpilib
%include "carrays.i"
%include "std_string.i"
%include "std_vector.i"
%{
#include <WPILib/WPILib.h>
%}

namespace std {
    %template(vector_charp) vector<const char *>;
    %template(vector_string) vector<string>;
}

typedef signed char INT8;
typedef signed short INT16;
typedef signed int INT32;
typedef signed long long INT64;

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef unsigned long long UINT64;

%array_functions(UINT8, UINT8array);

/*** ABSTRACT BASES ***/
class ErrorBase;

class Error
{
public:
    typedef tRioStatusCode Code;

    Error();
    ~Error();
    void Clone(Error &error);
    Code GetCode() const;
    const char *GetMessage() const;
    const char *GetFilename() const;
    const char *GetFunction() const;
    UINT32 GetLineNumber() const;
    const ErrorBase* GetOriginatingObject() const;
    double GetTime() const;
    void Clear();
    static void EnableStackTrace(bool enable) { m_stackTraceEnabled=enable; }
    static void EnableSuspendOnError(bool enable) { m_suspendOnErrorEnabled=enable; }
};

class ErrorBase
{
public:
    virtual ~ErrorBase();
    virtual Error& GetError();
    virtual void ClearError();
    virtual bool StatusIsFatal() const;
    static Error& GetGlobalError();
protected:
    ErrorBase();
};

class SensorBase: public ErrorBase
{
public:
    SensorBase();
    virtual ~SensorBase();
    static void DeleteSingletons();
    static UINT32 GetDefaultAnalogModule() { return 1; }
    static UINT32 GetDefaultDigitalModule() { return 1; }
    static UINT32 GetDefaultSolenoidModule() { return 1; }
    static bool CheckAnalogModule(UINT8 moduleNumber);
    static bool CheckDigitalModule(UINT8 moduleNumber);
    static bool CheckPWMModule(UINT8 moduleNumber);
    static bool CheckRelayModule(UINT8 moduleNumber);
    static bool CheckSolenoidModule(UINT8 moduleNumber);
    static bool CheckDigitalChannel(UINT32 channel);
    static bool CheckRelayChannel(UINT32 channel);
    static bool CheckPWMChannel(UINT32 channel);
    static bool CheckAnalogChannel(UINT32 channel);
    static bool CheckSolenoidChannel(UINT32 channel);

    static const UINT32 kSystemClockTicksPerMicrosecond = 40;
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
    virtual void RequestInterrupts() = 0;        ///< Synchronus Wait version.
    virtual void CancelInterrupts();            ///< Free up the underlying chipobject functions.
    virtual void WaitForInterrupt(float timeout); ///< Synchronus version.
    virtual void EnableInterrupts();            ///< Enable interrupts - after finishing setup.
    virtual void DisableInterrupts();        ///< Disable, but don't deallocate.
    virtual double ReadInterruptTimestamp();        ///< Return the timestamp for the interrupt that occurred.
};

class Sendable
{
public:
	//virtual void InitTable(ITable* subtable) = 0;
	//virtual ITable* GetTable() = 0;
	virtual std::string GetSmartDashboardType() = 0;
};

class LiveWindowSendable: public Sendable
{
public:
    virtual void UpdateTable() = 0;
    virtual void StartLiveWindowMode() = 0;
    virtual void StopLiveWindowMode() = 0;
};

class I2C : private SensorBase
{
    friend class DigitalModule;
public:
    virtual ~I2C();
    bool Transaction(UINT8 *dataToSend, UINT8 sendSize, UINT8 *dataReceived, UINT8 receiveSize);
    bool AddressOnly();
    bool Write(UINT8 registerAddress, UINT8 data);
    bool Read(UINT8 registerAddress, UINT8 count, UINT8 *data);
    void Broadcast(UINT8 registerAddress, UINT8 data);
    void SetCompatibilityMode(bool enable);

    bool VerifySensor(UINT8 registerAddress, UINT8 count, const UINT8 *expected);
private:
    I2C(DigitalModule *module, UINT8 deviceAddress);
};

class DigitalSource: public InterruptableSensorBase
{
public:
    virtual ~DigitalSource();
    virtual UINT32 GetChannelForRouting() = 0;
    virtual UINT32 GetModuleForRouting() = 0;
    virtual bool GetAnalogTriggerForRouting() = 0;
    virtual void RequestInterrupts() = 0;
    //virtual void RequestInterrupts(tInterruptHandler handler, void *param);
};

class SolenoidBase : public SensorBase {
public:
    virtual ~SolenoidBase();
    UINT8 GetAll();

protected:
    explicit SolenoidBase(UINT32 slot);
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

class DashboardBase : public ErrorBase {
public:
    virtual void GetStatusBuffer(char** userStatusData, INT32* userStatusDataSize) = 0;
    virtual void Flush() = 0;
    virtual ~DashboardBase() {}
protected:
    DashboardBase() {}
};

class PWM : public SensorBase, public LiveWindowSendable
{
public:
    typedef enum {kPeriodMultiplier_1X = 1, kPeriodMultiplier_2X = 2, kPeriodMultiplier_4X = 4} PeriodMultiplier;

    explicit PWM(UINT32 channel);
    PWM(UINT32 slot, UINT32 channel);
    virtual ~PWM();
    virtual void SetRaw(UINT8 value);
    virtual UINT8 GetRaw();
    void SetPeriodMultiplier(PeriodMultiplier mult);
    void EnableDeadbandElimination(bool eliminateDeadband);
    void SetBounds(INT32 max, INT32 deadbandMax, INT32 center, INT32 deadbandMin, INT32 min);
    UINT32 GetChannel();
    UINT32 GetModuleNumber();
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

class PIDOutput
{
public:
    virtual void PIDWrite(float output) = 0;
};

class PIDSource
{
public:
    virtual double PIDGet() = 0;
};

class SpeedController : public PIDOutput
{
public:
    virtual ~SpeedController() {};
    virtual void Set(float speed) = 0;
    virtual float Get() = 0;
    virtual void Disable() = 0;
};

class Module: public SensorBase
{
public:
    //nLoadOut::tModuleType GetType() {return m_moduleType;}
    UINT8 GetNumber() {return m_moduleNumber;}
    static Module *GetModule(nLoadOut::tModuleType type, UINT8 number);

protected:
    explicit Module(UINT32 slot);
    virtual ~Module();
};

class MotorSafety
{
public:
    virtual void SetExpiration(float timeout) = 0;
    virtual float GetExpiration() = 0;
    virtual bool IsAlive() = 0;
    virtual void StopMotor() = 0;
    virtual void SetSafetyEnabled(bool enabled) = 0;
    virtual bool IsSafetyEnabled() = 0;
    //virtual void GetDescription(char *desc) = 0;
};

class MotorSafetyHelper {
public:
    MotorSafetyHelper(MotorSafety *safeObject);
    ~MotorSafetyHelper();
    void Feed();
    void SetExpiration(float expirationTime);
    float GetExpiration();
    bool IsAlive();
    void Check();
    void SetSafetyEnabled(bool enabled);
    bool IsSafetyEnabled();
    static void CheckMotors();
};

class SafePWM: public PWM, public MotorSafety {
public:
    explicit SafePWM(UINT32 channel);
    SafePWM(UINT32 slot, UINT32 channel);
    
    void SetExpiration(float timeout);
    float GetExpiration();
    bool IsAlive();
    void StopMotor();
    bool IsSafetyEnabled();
    void SetSafetyEnabled(bool enabled);

    virtual void SetSpeed(float speed);
};

/*** CONCRETE CLASSES ***/

class ADXL345_I2C : public SensorBase
{
public:
    enum DataFormat_Range {kRange_2G=0x00, kRange_4G=0x01, kRange_8G=0x02, kRange_16G=0x03};
    enum Axes {kAxis_X=0x00, kAxis_Y=0x02, kAxis_Z=0x04};

    explicit ADXL345_I2C(UINT32 slot, DataFormat_Range range=kRange_2G);
    virtual ~ADXL345_I2C();
    double GetAcceleration(Axes axis);
};

class Accelerometer : public SensorBase, public PIDSource, public LiveWindowSendable
{
public:
    explicit Accelerometer(UINT32 channel);
    Accelerometer(UINT32 slot, UINT32 channel);
    explicit Accelerometer(AnalogChannel *channel);
    virtual ~Accelerometer();

    float GetAcceleration();
    void SetSensitivity(float sensitivity);
    void SetZero(float zero);
    double PIDGet();

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class AnalogChannel : public SensorBase, public PIDSource, public LiveWindowSendable
{
public:
    static const UINT8 kAccumulatorModuleNumber = 1;
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

    UINT8 GetModuleNumber();
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
    
    double PIDGet();

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class AnalogModule: public Module
{ 
public:
    static const long kTimebase = 40000000; ///< 40 MHz clock
    static const long kDefaultOversampleBits = 0;
    static const long kDefaultAverageBits = 7;
    static const float kDefaultSampleRate = 50000.0;

    void SetSampleRate(float samplesPerSecond);
    float GetSampleRate();
    void SetAverageBits(UINT32 channel, UINT32 bits);
    UINT32 GetAverageBits(UINT32 channel);
    void SetOversampleBits(UINT32 channel, UINT32 bits);
    UINT32 GetOversampleBits(UINT32 channel);
    INT16 GetValue(UINT32 channel);
    INT32 GetAverageValue(UINT32 channel);
    float GetAverageVoltage(UINT32 channel);
    float GetVoltage(UINT32 channel);
    UINT32 GetLSBWeight(UINT32 channel);
    INT32 GetOffset(UINT32 channel);
    INT32 VoltsToValue(INT32 channel, float voltage);

    static AnalogModule* GetInstance(UINT8 moduleNumber);

protected:
    explicit AnalogModule(UINT32 slot);
    virtual ~AnalogModule();
};

class AnalogTrigger: public SensorBase
{
    friend class AnalogTriggerOutput;
public:
    AnalogTrigger(UINT32 slot, UINT32 channel);
    explicit AnalogTrigger(UINT32 channel);
    explicit AnalogTrigger(AnalogChannel *channel);
    virtual ~AnalogTrigger();

    void SetLimitsVoltage(float lower, float upper);
    void SetLimitsRaw(INT32 lower, INT32 upper);
    void SetAveraged(bool useAveragedValue);
    void SetFiltered(bool useFilteredValue);
    UINT32 GetIndex();
    bool GetInWindow();
    bool GetTriggerState();
    AnalogTriggerOutput *CreateOutput(AnalogTriggerOutput::Type type);
};

class AnalogTriggerOutput: public DigitalSource
{
    friend class AnalogTrigger;
public:
    typedef enum {kInWindow=0, kState=1, kRisingPulse=2, kFallingPulse=3} Type;
    
    virtual ~AnalogTriggerOutput();
    bool Get();

    // DigitalSource interface
    virtual UINT32 GetChannelForRouting();
    virtual UINT32 GetModuleForRouting();
    virtual bool GetAnalogTriggerForRouting();
    virtual void RequestInterrupts();        ///< Synchronus Wait version.
protected:
    AnalogTriggerOutput(AnalogTrigger *trigger, Type outputType);
};

class Compressor: public SensorBase, public LiveWindowSendable
{
public:
    Compressor(UINT32 pressureSwitchChannel, UINT32 compressorRelayChannel);
    Compressor(UINT32 pressureSwitchSlot, UINT32 pressureSwitchChannel,
                UINT32 compresssorRelaySlot, UINT32 compressorRelayChannel);
    ~Compressor();

    void Start();
    void Stop();
    bool Enabled();
    UINT32 GetPressureSwitchValue();
    void SetRelayValue(Relay::Value relayValue);

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class Controller
{
public:
    virtual ~Controller() {};
    virtual void Enable() = 0;
    virtual void Disable() = 0;
};

class Counter : public SensorBase, public CounterBase, public LiveWindowSendable
{
public:
    typedef enum {kTwoPulse=0, kSemiperiod=1, kPulseLength=2, kExternalDirection=3} Mode;

    Counter();
    explicit Counter(UINT32 channel);
    Counter(UINT32 slot, UINT32 channel);
    explicit Counter(DigitalSource *source);
    explicit Counter(AnalogTrigger *trigger);
    Counter(EncodingType encodingType, DigitalSource *upSource, DigitalSource *downSource, bool inverted);
    virtual ~Counter();

    void SetUpSource(UINT32 channel);
    void SetUpSource(UINT32 slot, UINT32 channel);
    void SetUpSource(AnalogTrigger *analogTrigger, AnalogTriggerOutput::Type triggerType);
    void SetUpSource(DigitalSource *source);
    void SetUpSourceEdge(bool risingEdge, bool fallingEdge);
    void ClearUpSource();

    void SetDownSource(UINT32 channel);
    void SetDownSource(UINT32 slot, UINT32 channel);
    void SetDownSource(AnalogTrigger *analogTrigger, AnalogTriggerOutput::Type triggerType);
    void SetDownSource(DigitalSource *source);
    void SetDownSourceEdge(bool risingEdge, bool fallingEdge);
    void ClearDownSource();

    void SetUpDownCounterMode();
    void SetExternalDirectionMode();
    void SetSemiPeriodMode(bool highSemiPeriod);
    void SetPulseLengthMode(float threshold);

    void SetReverseDirection(bool reverseDirection);

    // CounterBase interface
    void Start();
    INT32 Get();
    void Reset();
    void Stop();
    double GetPeriod();
    void SetMaxPeriod(double maxPeriod);
    void SetUpdateWhenEmpty(bool enabled);
    bool GetStopped();
    bool GetDirection();
    UINT32 GetIndex() {return m_index;}

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    virtual std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class Dashboard : public DashboardBase
{
public:
    explicit Dashboard(SEM_ID statusDataSemaphore);
    virtual ~Dashboard();

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
    void GetStatusBuffer(char** userStatusData, INT32* userStatusDataSize);
    void Flush() {}
};

%rename(Get) DigitalInput::GetBool;
%rename(GetInt) DigitalInput::Get;

class DigitalInput : public DigitalSource, public LiveWindowSendable
{
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
    virtual void RequestInterrupts();        ///< Synchronus Wait version.
    void SetUpSourceEdge(bool risingEdge, bool fallingEdge);
    
    %extend
    {
        bool GetBool()
        {
            return $self->Get() != 0;
        }
    }

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class DigitalModule: public Module
{
    friend class I2C;
    friend class Module;

protected:
    explicit DigitalModule(UINT32 slot);
    virtual ~DigitalModule();

public:
    void SetPWM(UINT32 channel, UINT8 value);
    UINT8 GetPWM(UINT32 channel);
    void SetPWMPeriodScale(UINT32 channel, UINT32 squelchMask);
    void SetRelayForward(UINT32 channel, bool on);
    void SetRelayReverse(UINT32 channel, bool on);
    bool GetRelayForward(UINT32 channel);
    UINT8 GetRelayForward();
    bool GetRelayReverse(UINT32 channel);
    UINT8 GetRelayReverse();
    bool AllocateDIO(UINT32 channel, bool input);
    void FreeDIO(UINT32 channel);
    void SetDIO(UINT32 channel, short value);
    bool GetDIO(UINT32 channel);
    UINT16 GetDIO();
    bool GetDIODirection(UINT32 channel);
    UINT16 GetDIODirection();
    void Pulse(UINT32 channel, float pulseLength);
    bool IsPulsing(UINT32 channel);
    bool IsPulsing();
    UINT32 AllocateDO_PWM();
    void FreeDO_PWM(UINT32 pwmGenerator);
    void SetDO_PWMRate(float rate);
    void SetDO_PWMDutyCycle(UINT32 pwmGenerator, float dutyCycle);
    void SetDO_PWMOutputChannel(UINT32 pwmGenerator, UINT32 channel);

    I2C* GetI2C(UINT32 address);

    static DigitalModule* GetInstance(UINT8 moduleNumber);
    static UINT8 RemapDigitalChannel(UINT32 channel) { return 15 - channel; }; // TODO: Need channel validation
    static UINT8 UnmapDigitalChannel(UINT32 channel) { return 15 - channel; }; // TODO: Need channel validation
};

class DigitalOutput : public DigitalSource, public LiveWindowSendable
{
public:
    explicit DigitalOutput(UINT32 channel);
    DigitalOutput(UINT32 slot, UINT32 channel);
    virtual ~DigitalOutput();
    void Set(UINT32 value);
    void Pulse(float length);
    bool IsPulsing();
    void SetPWMRate(float rate);
    void EnablePWM(float initialDutyCycle);
    void DisablePWM();
    void UpdateDutyCycle(float dutyCycle);

    // Digital Source Interface
    virtual UINT32 GetChannelForRouting();
    virtual UINT32 GetModuleForRouting();
    virtual bool GetAnalogTriggerForRouting();
    virtual void RequestInterrupts();

    void SetUpSourceEdge(bool risingEdge, bool fallingEdge);
    
    //virtual void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class DoubleSolenoid : public SolenoidBase, public LiveWindowSendable {
public:
    typedef enum {kOff, kForward, kReverse} Value;

    explicit DoubleSolenoid(UINT32 forwardChannel, UINT32 reverseChannel);
    DoubleSolenoid(UINT32 slot, UINT32 forwardChannel, UINT32 reverseChannel);
    virtual ~DoubleSolenoid();
    virtual void Set(Value value);
    virtual Value Get();

    //void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class DriverStation : public SensorBase
{
public:
    enum Alliance {kRed, kBlue, kInvalid};

    virtual ~DriverStation();
    static DriverStation *GetInstance();

    static const UINT32 kBatteryModuleNumber = 1;
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
    void WaitForData();
    double GetMatchTime();
    float GetBatteryVoltage();
    UINT16 GetTeamNumber();

    Dashboard& GetHighPriorityDashboardPacker(void) {return m_dashboardHigh;}
    Dashboard& GetLowPriorityDashboardPacker(void) {return m_dashboardLow;}
    DashboardBase* GetHighPriorityDashboardPackerInUse() { return m_dashboardInUseHigh; }
    DashboardBase* GetLowPriorityDashboardPackerInUse() { return m_dashboardInUseLow; }
    void SetHighPriorityDashboardPackerToUse(DashboardBase* db) { m_dashboardInUseHigh = db; }
    void SetLowPriorityDashboardPackerToUse(DashboardBase* db) { m_dashboardInUseLow = db; }
    DriverStationEnhancedIO& GetEnhancedIO(void) {return m_enhancedIO;}
    void IncrementUpdateNumber() { m_updateNumber++; }
    SEM_ID GetUserStatusDataSem() { return m_statusDataSemaphore; }

    void InDisabled(bool entering) {m_userInDisabled=entering;}
    void InAutonomous(bool entering) {m_userInAutonomous=entering;}
    void InOperatorControl(bool entering) {m_userInTeleop=entering;}
    void InTest(bool entering) {m_userInTest=entering;}
protected:
    DriverStation();
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
private:
    DriverStationEnhancedIO();
    virtual ~DriverStationEnhancedIO();
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

protected:
    DriverStationLCD();
};

class Encoder: public SensorBase, public CounterBase, public PIDSource, public LiveWindowSendable
{
public:
    typedef enum {kDistance, kRate} PIDSourceParameter;

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

    void SetPIDSourceParameter(PIDSourceParameter pidSource);
    double PIDGet();
    
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class GearTooth : public Counter
{
public:
    /// 55 uSec for threshold
    static const double kGearToothThreshold = 55e-6;
    GearTooth(UINT32 slot, UINT32 channel, bool directionSensitive);
    GearTooth(DigitalSource *source, bool directionSensitive);
    virtual ~GearTooth();
    void EnableDirectionSensing(bool directionSensitive);

    virtual std::string GetSmartDashboardType();
};

class Gyro : public SensorBase, public PIDSource, public LiveWindowSendable
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
    virtual float GetAngle();
    void SetSensitivity(float voltsPerDegreePerSecond);
    virtual void Reset();
    
    double PIDGet();

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class HiTechnicColorSensor : public SensorBase
{
public:
    enum tColorMode {kActive = 0, kPassive = 1, kRaw = 3};
    //typedef struct{
    //    UINT16 red;
    //    UINT16 blue;
    //    UINT16 green;
    //}RGB;
    explicit HiTechnicColorSensor(UINT8 moduleNumber);
    virtual ~HiTechnicColorSensor();
    UINT8 GetColor();
    UINT8 GetRed();
    UINT8 GetGreen();
    UINT8 GetBlue();
    //RGB GetRGB();
    UINT16 GetRawRed();
    UINT16 GetRawGreen();
    UINT16 GetRawBlue();
    //RGB GetRawRGB();
    void SetMode(tColorMode mode);


    //LiveWindowSendable interface
    virtual std::string GetType();
    virtual void UpdateTable();
    virtual void StartLiveWindowMode();
    virtual void StopLiveWindowMode(); 
    //virtual void InitTable(ITable *subtable);
    //virtual ITable* GetTable();
};

class HiTechnicCompass : public SensorBase
{
public:
    explicit HiTechnicCompass(UINT32 slot);
    virtual ~HiTechnicCompass();
    float GetAngle();

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class Jaguar : public SafePWM, public SpeedController
{
public:
    explicit Jaguar(UINT32 channel);
    Jaguar(UINT32 slot, UINT32 channel);
    virtual ~Jaguar();
    virtual float Get();
    virtual void Set(float value);
    virtual void Disable();
    
    void PIDWrite(float output);
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

class LiveWindow {
public:
    static LiveWindow * GetInstance();
    void Run();
    //void AddSensor(char *subsystem, char *name, LiveWindowSendable *component);
    //void AddActuator(char *subsystem, char *name, LiveWindowSendable *component);
    //void AddSensor(std::string type, int module, int channel, LiveWindowSendable *component);
    //void AddActuator(std::string type, int module, int channel, LiveWindowSendable *component);
    
    bool IsEnabled() { return m_enabled; }
    void SetEnabled(bool enabled);

protected:
    LiveWindow();
    virtual ~LiveWindow();
};

class NetworkTable : public ErrorBase {
public:
    static const char PATH_SEPARATOR_CHAR;
    static const std::string PATH_SEPARATOR;
    static const int DEFAULT_PORT;
    
    static void Initialize();
    static void SetServerMode();
    static void SetTeam(int team);
    static void SetIPAddress(const char* address);
    static NetworkTable* GetTable(std::string key);
    NetworkTable(std::string path, NetworkTableProvider& provider);
    virtual ~NetworkTable();
    bool IsConnected();
    bool IsServer();
    void AddConnectionListener(IRemoteConnectionListener* listener, bool immediateNotify);
    void RemoveConnectionListener(IRemoteConnectionListener* listener);
    void AddTableListener(ITableListener* listener);
    void AddTableListener(ITableListener* listener, bool immediateNotify);
    void AddTableListener(std::string key, ITableListener* listener, bool immediateNotify);
    void AddSubTableListener(ITableListener* listener);
    void RemoveTableListener(ITableListener* listener);
    NetworkTable* GetSubTable(std::string key);
    bool ContainsKey(std::string key);
    bool ContainsSubTable(std::string key);
    void PutNumber(std::string key, double value);
    double GetNumber(std::string key);
    double GetNumber(std::string key, double defaultValue);
    void PutString(std::string key, std::string value);
    std::string GetString(std::string key);
    std::string GetString(std::string key, std::string defaultValue);
    void PutBoolean(std::string key, bool value);
    bool GetBoolean(std::string key);
    bool GetBoolean(std::string key, bool defaultValue);
    void PutValue(std::string key, NetworkTableEntryType* type, EntryValue value);
    void RetrieveValue(std::string key, ComplexData& externalValue);
    void PutValue(std::string key, ComplexData& value);
    EntryValue GetValue(std::string key);
    EntryValue GetValue(std::string key, EntryValue defaultValue);
};

class PIDController : public LiveWindowSendable, public Controller
{
public:
    PIDController(float p, float i, float d,
                    PIDSource *source, PIDOutput *output,
                    float period = 0.05);
    PIDController(float p, float i, float d, float f,
                    PIDSource *source, PIDOutput *output,
                    float period = 0.05);
    virtual ~PIDController();
    virtual float Get();
    virtual void SetContinuous(bool continuous = true);
    virtual void SetInputRange(float minimumInput, float maximumInput);
    virtual void SetOutputRange(float mimimumOutput, float maximumOutput);
    virtual void SetPID(float p, float i, float d);
    virtual void SetPID(float p, float i, float d, float f);
    virtual float GetP();
    virtual float GetI();
    virtual float GetD();
    virtual float GetF();
    
    virtual void SetSetpoint(float setpoint);
    virtual float GetSetpoint();

    virtual float GetError();
    
    virtual void SetTolerance(float percent);
    virtual void SetAbsoluteTolerance(float absValue);
    virtual void SetPercentTolerance(float percentValue);
    virtual bool OnTarget();
    
    virtual void Enable();
    virtual void Disable();
    virtual bool IsEnabled();
    
    virtual void Reset();

    //virtual void InitTable(ITable* table);
};

class Preferences : public ErrorBase
{
public:
    static Preferences *GetInstance();

    std::vector<std::string> GetKeys();
    std::string GetString(const char *key, const char *defaultValue = "");
    int GetString(const char *key, char *value, int valueSize, const char *defaultValue = "");
    int GetInt(const char *key, int defaultValue = 0);
    double GetDouble(const char *key, double defaultValue = 0.0);
    float GetFloat(const char *key, float defaultValue = 0.0);
    bool GetBoolean(const char *key, bool defaultValue = false);
    INT64 GetLong(const char *key, INT64 defaultValue = 0);
    void PutString(const char *key, const char *value);
    void PutInt(const char *key, int value);
    void PutDouble(const char *key, double value);
    void PutFloat(const char *key, float value);
    void PutBoolean(const char *key, bool value);
    void PutLong(const char *key, INT64 value);
    void Save();
    bool ContainsKey(const char *key);
    void Remove(const char *key);

    //void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
protected:
	Preferences();
	~Preferences();
};

class Relay : public SensorBase, public LiveWindowSendable
{
public:
    typedef enum {kOff, kOn, kForward, kReverse} Value;
    typedef enum {kBothDirections, kForwardOnly, kReverseOnly} Direction;

    Relay(UINT32 slot, UINT32 channel, Direction direction);
    virtual ~Relay();

    void Set(Value value);
    Value Get();

    //void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class RobotDrive : public MotorSafety
{
public:
    typedef enum
    {
        kFrontLeftMotor = 0,
        kFrontRightMotor = 1,
        kRearLeftMotor = 2,
        kRearRightMotor = 3
    } MotorType;

    RobotDrive(UINT32 leftMotorChannel, UINT32 rightMotorChannel);
    RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
                UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
    RobotDrive(SpeedController *leftMotor, SpeedController *rightMotor);
    RobotDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
                SpeedController *frontRightMotor, SpeedController *rearRightMotor);
    virtual ~RobotDrive();

    void Drive(float outputMagnitude, float curve);
    void TankDrive(GenericHID *leftStick, GenericHID *rightStick);
    void TankDrive(GenericHID *leftStick, UINT32 leftAxis, GenericHID *rightStick, UINT32 rightAxis);
    void TankDrive(float leftValue, float rightValue);
    void ArcadeDrive(GenericHID *stick, bool squaredInputs = true);
    void ArcadeDrive(GenericHID *moveStick, UINT32 moveChannel, GenericHID *rotateStick, UINT32 rotateChannel, bool squaredInputs = true);
    void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs = true);
    void MecanumDrive_Cartesian(float x, float y, float rotation, float gyroAngle = 0.0);
    void MecanumDrive_Polar(float magnitude, float direction, float rotation);
    void HolonomicDrive(float magnitude, float direction, float rotation);
    virtual void SetLeftRightMotorOutputs(float leftOutput, float rightOutput);
    void SetInvertedMotor(MotorType motor, bool isInverted);
    void SetSensitivity(float sensitivity);
    void SetMaxOutput(double maxOutput);

    void SetExpiration(float timeout);
    float GetExpiration();
    bool IsAlive();
    void StopMotor();
    bool IsSafetyEnabled();
    void SetSafetyEnabled(bool enabled);
};

class SerialPort
{
public:
    typedef enum {kParity_None=0, kParity_Odd=1, kParity_Even=2, kParity_Mark=3, kParity_Space=4} Parity;
    typedef enum {kStopBits_One=10, kStopBits_OnePointFive=15, kStopBits_Two=20} StopBits;
    typedef enum {kFlowControl_None=0, kFlowControl_XonXoff=1, kFlowControl_RtsCts=2, kFlowControl_DtrDsr=4} FlowControl;
    typedef enum {kFlushOnAccess=1, kFlushWhenFull=2} WriteBufferMode;

    SerialPort(UINT32 baudRate, UINT8 dataBits = 8, Parity parity = kParity_None, StopBits stopBits = kStopBits_One);
    ~SerialPort();
    void SetFlowControl(FlowControl flowControl);
    void EnableTermination(char terminator = '\n');
    void DisableTermination();
    INT32 GetBytesReceived();
    UINT32 Read(char *buffer, INT32 count);
    UINT32 Write(const char *buffer, INT32 count);
    void SetTimeout(float timeout);
    void SetReadBufferSize(UINT32 size);
    void SetWriteBufferSize(UINT32 size);
    void SetWriteBufferMode(WriteBufferMode mode);
    void Flush();
    void Reset();
};

class Servo : public SafePWM
{
public:
    explicit Servo(UINT32 channel);
    Servo(UINT32 slot, UINT32 channel);
    virtual ~Servo();
    void Set(float value);
    void SetOffline();
    float Get();
    void SetAngle(float angle);
    float GetAngle();
    static float GetMaxAngle() { return kMaxServoAngle; };
    static float GetMinAngle() { return kMinServoAngle; };

    //void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class SmartDashboard : public SensorBase {
public:
    static void init();
    
    //static void PutData(std::string key, Sendable *data);
    //static void PutData(NamedSendable *value);
    //static Sendable* GetData(std::string keyName);
    
    static void PutBoolean(std::string keyName, bool value);
    static bool GetBoolean(std::string keyName);
    
    static void PutNumber(std::string keyName, double value);
    static double GetNumber(std::string keyName);
    
    static void PutString(std::string keyName, std::string value);
    static int GetString(std::string keyName, char *value, unsigned int valueLen);
    static std::string GetString(std::string keyName);
    
    static void PutValue(std::string keyName, ComplexData& value);
    static void RetrieveValue(std::string keyName, ComplexData& value);
private:
    SmartDashboard();
    virtual ~SmartDashboard();
};

class Solenoid : public SolenoidBase
{
public:
    explicit Solenoid(UINT32 channel);
    Solenoid(UINT32 slot, UINT32 channel);
    virtual ~Solenoid();
    virtual void Set(bool on);
    virtual bool Get();

    //void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew);
    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class Talon : public SafePWM, public SpeedController
{
public:
    explicit Talon(UINT32 channel);
    Talon(UINT8 moduleNumber, UINT32 channel);
    virtual ~Talon();
    virtual void Set(float value, UINT8 syncGroup=0);
    virtual float Get();
    virtual void Disable();

    virtual void PIDWrite(float output);
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
void Wait(double seconds);

class Ultrasonic: public SensorBase, public PIDSource, public LiveWindowSendable
{
public:
    typedef enum {
        kInches = 0,
        kMilliMeters = 1
    } DistanceUnit;
    
    Ultrasonic(DigitalOutput *pingChannel, DigitalInput *echoChannel, DistanceUnit units = kInches);
    Ultrasonic(UINT32 pingChannel, UINT32 echoChannel, DistanceUnit units = kInches);
    Ultrasonic(UINT32 pingSlot, UINT32 pingChannel, UINT32 echoSlot, UINT32 echoChannel, DistanceUnit units = kInches);
    virtual ~Ultrasonic();

    void Ping();
    bool IsRangeValid();
    static void SetAutomaticMode(bool enabling);
    double GetRangeInches();
    double GetRangeMM();
    bool IsEnabled() { return m_enabled; }
    void SetEnabled(bool enable) { m_enabled = enable; }
    
    double PIDGet();
    void SetDistanceUnits(DistanceUnit units);
    DistanceUnit GetDistanceUnits();

    void UpdateTable();
    void StartLiveWindowMode();
    void StopLiveWindowMode();
    std::string GetSmartDashboardType();
    //void InitTable(ITable *subTable);
    //ITable * GetTable();
};

class Victor : public SafePWM, public SpeedController
{
public:
    explicit Victor(UINT32 channel);
    Victor(UINT32 slot, UINT32 channel);
    virtual ~Victor();
    virtual void Set(float value);
    virtual float Get();
    virtual void Disable();
    
    virtual void PIDWrite(float output);
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

bool IsEnabled()
{
    return RobotBase::getInstance().IsEnabled();
}

bool IsDisabled()
{
    return RobotBase::getInstance().IsDisabled();
}

bool IsAutonomous()
{
    return RobotBase::getInstance().IsAutonomous();
}

bool IsOperatorControl()
{
    return RobotBase::getInstance().IsOperatorControl();
}

bool IsTest()
{
    return RobotBase::getInstance().IsTest();
}

bool IsSystemActive()
{
    return RobotBase::getInstance().IsSystemActive();
}

bool IsNewDataAvailable()
{
    return RobotBase::getInstance().IsNewDataAvailable();
}

Watchdog *GetWatchdog()
{
    return &RobotBase::getInstance().GetWatchdog();
}

%}
