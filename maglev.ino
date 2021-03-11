#define MIN_PWM_VALUE         0         //Minimum PWM duty cycle
#define MAX_PWM_VALUE         255       //Maximum PWM duty cycle

#define PID_UPDATE_INTERVAL   200        //PWM update interval, in microseconds. 
#define DEFAULT_TARGET_VALUE  210       //Default target hall effect readout
#define DEFAULT_KP            100     
#define DEFAULT_KD            20

int roundValue(float value)
{
  return (int)(value + 0.5);
}
float count = 0;
const int hallSensorPin = 0;
const int gMidpoint = roundValue((MAX_PWM_VALUE - MIN_PWM_VALUE) / 2); //The midpoint of our PWM range

signed int gNextPIDCycle = 0; ///KEEP AS SIGNED. Holds the next time we need to recalculate PID outputs. Uses overflow safe arithmetic so does not need to hold the entire output of millis(). Must be able to hold PID_UPDATE_INTERVAL without overflow
signed int gNextSineUpdate = 0; ///KEEP AS SIGNED. Holds the next time we need to recalculate PID outputs. Uses overflow safe arithmetic so does not need to hold the entire output of millis(). Must be able to hold PID_UPDATE_INTERVAL without overflow
int gCurrentDutyCycle = 0; //Current PWM duty cycle for the coil
int gLastSensorReadout = 0; //Last sensor readout to calculate derivative term
int gNextSensorReadout = 0; //The "next" sensor value. Declared global so we can use it as a running average and move ot to gLastSensorReadout after PWM calculation

int gTargetValue = DEFAULT_TARGET_VALUE;
float gKp = DEFAULT_KP;
float gKd = DEFAULT_KD;

void writeCoilPWM(int value)
{
    OCR1B = value; 
}


void setup()
{  
    // Setup timer 1 as Phase Correct non-inverted PWM, 31372.55 Hz.
    pinMode(12, OUTPUT);
    pinMode(11, OUTPUT);
    // WGM20 is used for Phase Correct PWM, COM2A1/COM2B1 sets output to non-inverted
    TCCR1A = 0;
    TCCR1A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    // PWM frequency is 16MHz/255/2/<prescaler>, prescaler is 1 here by using CS20
    TCCR1B = 0;
    TCCR1B = _BV(CS20);

    pinMode(hallSensorPin, INPUT);
}


//Used while in active mode
void controlLoop()
{
  //Downcast micros to the type of the stored cycle argument, then calculate the difference as a signed number. If negative, the time hasn't passed yet. This allows us to do overflow-safe arithmetic
  if(0 <= ((typeof(gNextPIDCycle))micros() - gNextPIDCycle))
  {
    count = count+0.0005;
    gTargetValue = 60*sin(2*PI*count)+190;
    
    //By default, downcast to signed 16-bit int (safe), but can be changed to use longer intervals by changing the type of gNextPWMCycle
    gNextPIDCycle = micros() + PID_UPDATE_INTERVAL;
    
    //Read the sensor at least once in an update cycle
    gNextSensorReadout = analogRead(hallSensorPin);
    //gNextSensorReadout = roundValue(((gNextSensorReadout * (FILTERFACTOR - 1)) + analogRead(hallSensorPin)) / FILTERFACTOR); //USED TO ATTENUATE HOW RESPONSIVE THE SENSOR VALUE IS TO CHANGE IN SIGNAL
    int error = gTargetValue - gNextSensorReadout; //Difference between current and expected values (for proportional term)
    if(error > 120 || error < -120)
    {
      //Slope of the input over time (for derivative term). (Time intterval is contant, so there is no need to divide by elapsed time. The derivative gain coefficient can take care of the constant)
      int dError = (gNextSensorReadout - gLastSensorReadout); 
          
      gCurrentDutyCycle = gMidpoint - roundValue((gKp*error) - (gKd*dError));
      //It's possible to overshoot in the above, so constrain to between our max and min
      gCurrentDutyCycle = constrain(gCurrentDutyCycle, MIN_PWM_VALUE, MAX_PWM_VALUE);
      writeCoilPWM(gCurrentDutyCycle);
          
      //Store for next calculation of dError
      gLastSensorReadout = gNextSensorReadout;
    }
  }
}

void loop()
{
  controlLoop();      
}
