#include "Arduino.h"
#include "Pump.h"

//Constructor
//PumpPin is the Arduino relay output pin number to be switched to start/stop the pump
//TankLevelPin is the Arduino digital input pin number connected to the tank level switch
//Interlockpin is the Arduino digital input number connected to an "interlock". 
//If this input is LOW, pump is stopped and/or cannot start. This is used for instance to stop
//the Orp or pH pumps in case filtration pump is not running
//IsRunningSensorPin is the pin which is checked to know whether the pump is running or not. 
//It can be the same pin as "PumpPin" in case there is no sensor on the pump (pressure, current, etc) which is not as robust. 
//This option is especially useful in the case where the filtration pump is not managed by the Arduino. 
//FlowRate is the flow rate of the pump in Liters/Hour, typically 1.5 or 3.0 L/hour for peristaltic pumps for pools. This is used to compute how much of the tank we have emptied out
//TankVolume is used here to compute the percentage fill used
Pump::Pump(uint8_t PumpPin, uint8_t IsRunningSensorPin, uint8_t TankLevelPin, 
           uint8_t Interlockpin, double FlowRate, double TankVolume, double TankFill)
{
  pumppin = PumpPin;
  isrunningsensorpin = IsRunningSensorPin;
  tanklevelpin = TankLevelPin;
  interlockpin = Interlockpin;
  flowrate = FlowRate; //in Liters per hour
  tankvolume = TankVolume; //in Liters
  tankfill = TankFill; // in percent
  StartTime = 0;
  LastStartTime = 0;
  StopTime = 0;
  UpTime = 0;        
  UpTimeError = 0;
  MaxUpTime = DefaultMaxUpTime;
  CurrMaxUpTime = MaxUpTime;
}     

//Call this in the main loop, for every loop, as often as possible
void Pump::loop()
{
  if(digitalRead(isrunningsensorpin) == PUMP_ON)
  {
    UpTime += millis() - StartTime;
    StartTime = millis();
  }

  if((CurrMaxUpTime > 0) && (UpTime >= CurrMaxUpTime))
  {
    Stop();
    UpTimeError = true;
  }

  if(!this->Pump::TankLevel()) this->Pump::Stop();

  if(interlockpin != NO_INTERLOCK)
  {
    if(digitalRead(interlockpin) == INTERLOCK_NOK)
       Stop();
  }
}

//Switch pump ON if over time was not reached, tank is not empty and interlock is OK
bool Pump::Start()
{
  if((digitalRead(isrunningsensorpin) == PUMP_OFF) 
    && !UpTimeError
    && this->Pump::TankLevel()
    && ((interlockpin == NO_INTERLOCK) || (digitalRead(interlockpin) == INTERLOCK_OK)))    //if((digitalRead(pumppin) == false))
  {
    digitalWrite(pumppin, PUMP_ON);
    StartTime = LastStartTime = millis(); 
    return true; 
  }
  else return false;
}

//Switch pump OFF
bool Pump::Stop()
{
  if(digitalRead(isrunningsensorpin) == PUMP_ON)
  {
    digitalWrite(pumppin, PUMP_OFF);
    UpTime += millis() - StartTime; 
    return true;
  }
  else return false;
}

//Reset the tracking of running time
//This is typically called every day at midnight
void Pump::ResetUpTime()
{
  StartTime = 0;
  StopTime = 0;
  UpTime = 0;
  CurrMaxUpTime = MaxUpTime;
}

//Set a maximum running time (in millisecs) per day (in case ResetUpTime() is called once per day)
//Once reached, pump is stopped and "UpTimeError" error flag is raised
//Set "Max" to 0 to disable limit
void Pump::SetMaxUpTime(unsigned long Max)
{
  MaxUpTime = Max;
  CurrMaxUpTime = MaxUpTime;
}

//Clear "UpTimeError" error flag and allow the pump to run for an extra MaxUpTime
void Pump::ClearErrors()
{
  if(UpTimeError)
  {
    CurrMaxUpTime += MaxUpTime;
    UpTimeError = false;
  }
}

//tank level status (true = full, false = empty)
bool Pump::TankLevel()
{
  if(tanklevelpin == NO_TANK)
  {
    return true;
  }
  else if (tanklevelpin == NO_LEVEL)
  {
    return (this->Pump::GetTankFill() > 5.); //alert below 5% 
  }
  else
  {
    return (digitalRead(tanklevelpin) == TANK_FULL);
  } 
}

//Return the percentage used since last reset of UpTime
double Pump::GetTankUsage() 
{
  float PercentageUsed = -1.0;
  if((tankvolume != 0.0) && (flowrate !=0.0))
  {
    double MinutesOfUpTime = (double)UpTime/1000.0/60.0;
    double Consumption = flowrate/60.0*MinutesOfUpTime;
    PercentageUsed = Consumption/tankvolume*100.0;
  }
  return (PercentageUsed);  
}

//Return the remaining quantity in tank in %. When resetting UpTime, SetTankFill must be called accordingly
double Pump::GetTankFill()
{
  return (tankfill - this->Pump::GetTankUsage());
}

//Set Tank volume
//Typically call this function when changing tank and set it to the full volume
void Pump::SetTankVolume(double Volume)
{
  tankvolume = Volume;
}

//Set flow rate of the pump in Liters/hour
void Pump::SetFlowRate(double FlowRate)
{
  flowrate = FlowRate;
}

//Set tank fill (percentage of tank volume)
void Pump::SetTankFill(double TankFill)
{
  tankfill = TankFill;
}

//interlock status
bool Pump::Interlock()
{
  return (digitalRead(interlockpin) == INTERLOCK_OK);
}

//pump status
bool Pump::IsRunning()
{
  return (digitalRead(isrunningsensorpin) == PUMP_ON);
}
