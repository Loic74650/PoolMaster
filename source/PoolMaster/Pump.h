/*
            Pump - a simple library to handle home-pool filtration and persitaltic pumps
                        (c) Loic74 <loic74650@gmail.com> 2018
Features: 

- keeps track of running time
- keeps track of Tank Level
- set max running time limit

NB: all timings are in milliseconds
*/

 

#ifndef PUMP_h
#define PUMP_h
#define PUMP_VERSION "0.0.1"

//Constants used in some of the functions below
#define PUMP_ON  1
#define PUMP_OFF 0
#define TANK_FULL  1
#define TANK_EMPTY 0
#define NO_TANK 255   
 
class Pump{
  public:

  
    Pump(uint8_t, uint8_t);
    void loop();
    bool Start();
    bool Stop();
    bool IsRunning();
    bool TankLevel();
    void SetMaxUpTime(unsigned long Max);
    void ResetUpTime();
    void ClearErrors();
    
    unsigned long UpTime;
    unsigned long MaxUpTime;
    bool UpTimeError;
    unsigned long StartTime;
    unsigned long StopTime; 
          
  private:
     
    uint8_t pumppin; 
    uint8_t tanklevelpin;
};
#endif
