//ctre my beloved
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>


namespace motors
{
  enum motor_type
  {
    DRIVE,// go some speed
    ANGLE // snap to angle
  }

  class TalonFX
  {
    public:
      TalonFX(int CAN, motors::motor_type Mode);

      TalonFX();

      GetEncoder(int CAN);

      AddMotor(int CAN, motors::motor_type Mode);

      Config(float P, float I = 0, float D, int max_amperage);

      Set(double input);

      GetSpeed();

      GetMotorPosition();
    private:
        // initialize motor and encoder as unique pointers. These hold the address to a value with the ownership of it being limitied to this class.
        // unique_ptr is a type of shared pointer, which is very similar to a raw pointer but its designed to follow intended ownership semantics
        std::unique_ptr<ctre::phoenix6::hardware::TalonFX>  talonMotor;
        std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
        
        frc::PIDController angleController{1, 0, 0}; // why is this here? IG it doesnt need to be, OH SHOOT NVM we've got this beut' of a optimization
        motors::motor_type talonMode;

        int CANID;
  }
} // motors

// Constructor adds Motor and what mode its in
motors::TalonFX::TalonFX(int CAN, motors::motor_type Mode)
{ 
    this->talonMode  = Mode;
    this->CANID      = CAN;
    // Actual motor
    this->talonMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CAN, "rio");
};

// Empty constructor, this is meant to be used in conjunction with AddMotor so you dont need to initialize any code in a header.
motors::TalonFX::TalonFX() 
{
};

/// @brief gets a CANcoder bc they are great
void motors::TalonFX::GetEncoder(int CAN)
{
    this->encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(CAN, "rio");
};

// Meant to be used with the empty contructor, this does what the actual constructor does. This could also be used to convert one motor to something else. You SHOULD NOT use this to switch modes, only motors
void motors::TalonFX::AddMotor(int CAN, motors::motor_type Mode)
{
    this->talonMode = Mode;
    this->CANID     = CAN;
    // Actual motor
    this->talonMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CAN, "rio");
};

/// @brief adds PID and amperage limiters to motor. This MUST be used.
void motors::TalonFX::Config(float P, float I = 0, float D = 0, int max_amperage)
{
    // *** set PID
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{}; // initiallize the configurator
    // set slot 0 gains and leave every other config factory-default
    ctre::phoenix6::configs::Slot0Configs& slot0Configs = talonFXConfigs.Slot0; // use slot0 in  for configurator, you can use other slots
    slot0Configs.kP = P;
    slot0Configs.kI = I;
    slot0Configs.kD = D;

    // apply configs, 50 ms total timeout
    this->talonMotor.get()->GetConfigurator().Apply(talonFXConfigs, 50_ms);

    // *** Set the amperage current limit
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{}; // initialize the configurator
    currentLimitsConfigs.StatorCurrentLimit       = max_amperage; // set out maximum amperage
    currentLimitsConfigs.StatorCurrentLimitEnable = true; // enables the limiter
    this->talonMotor.get()->GetConfigurator().Apply(currentLimitsConfigs); // apply our configurator
}

/// @brief When in DRIVE mode it outputs 1,-1 to the motor; when in ANGLE mode you should give it an angle
void motors::TalonFX::Set(double input)  
{
    switch (this->talonMode)
    {
    case motors::motor_type::DRIVE:
        this->talonMotor.get()->Set(input); // takes the input (should be 1,-1) and outputs it to the motor
    case motors::motor_type::ANGLE:
        this->talonMotor.get()->Set(this->angleController.Calculate(this->GetMotorPosition(), input)); // take the input (0,360), runs it through a PID feedback loop
    default:
        this->CANID = 69;
    }
}

/// @brief just the current set speed
/// @returns last setSpeed() input -1.0 - 1
double motors::TalonFX::GetSpeed()  
{
    return this->talonMotor.get()->Get();
}

/// @brief Gets the angle from either the encoder (if present) or the motor.
/// @return position/distance driven in degrees
double motors::TalonFX::GetMotorPosition()  
{
  // checks if encoder exists
  if (this->encoder == null)
  {
    // returns angle as a double from motor
    return units::unit_cast<double, units::angle::turn_t>(this->talonMotor.get()->GetPosition().GetValue() * 360); // converts rotations to degrees
  } else
  {
    // same but from the encoder
    return units::unit_cast<double, units::angle::turn_t>(this->encoder.get()->GetPosition().GetValue() * 360);
  }
}
