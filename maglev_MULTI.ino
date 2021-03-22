#define MIN_PWM_VALUE                   0         //Minimum PWM duty cycle
#define MAX_PWM_VALUE                   255       //Maximum PWM duty cycle 
#define DEFAULT_TARGET_VALUE            310       //Default target hall effect readout
#define DEFAULT_KP                      100    
#define DEFAULT_KD                      30
#define PROPORTIONAL_THRESHOLD          40

int roundValue(float value)
{
  return (int)(value + 0.5);
}

int THRESHOLDP = PROPORTIONAL_THRESHOLD;
int THRESHOLDN = -1*PROPORTIONAL_THRESHOLD;
float count = 0;
const int hallSensorPin1 = 1;
const int hallSensorPin0 = 0;
const int gMidpoint = roundValue((MAX_PWM_VALUE - MIN_PWM_VALUE) / 2); //The midpoint of our PWM range
int gCurrentDutyCycle1 = 0; //Current PWM duty cycle for the coil
int gLastSensorReadout1 = 0; //Last sensor readout to calculate derivative term
int gNextSensorReadout1 = 0; //The "next" sensor value. 
int gCurrentDutyCycle0 = 0; //Current PWM duty cycle for the coil
int gLastSensorReadout0 = 0; //Last sensor readout to calculate derivative term
int gNextSensorReadout0 = 0; //The "next" sensor value. 
int gTargetValue1 = 275;
int gTargetValue0 = 275;
float gKp = DEFAULT_KP;
float gKd = DEFAULT_KD;

void writeCoilPWM(int value1, int value2)
{
    OCR1B = value1; 
    OCR0A = value2;
}

void setup()
{  
  //-------------------SETTING UP TIMER INTERRUPT FOR SAMPLE RATE-------------------------------
     noInterrupts(); //no interrupts during setup of timer
     TCCR4A = 0;
     TCCR4B = 0;
     TCNT4  = 0;//initialize counter value to 0
     // set compare match register for approximately 5Khz increments
     OCR4A = 50;
     // turn on CTC mode
     TCCR4B |= (1 << WGM12);
     // Set CS12 and CS10 bits for 64 prescaler
     TCCR4B |= (1 << CS11) | (1 << CS10);  
     // enable timer compare interrupt
     TIMSK4 |= (1 << OCIE4A);
    interrupts();//allow interrupts
 //-----------------SETTING UP TIMER FOR FIRST PWM----------------------------------------------------------------- 
    GTCCR = _BV(TSM) | _BV(PSRSYNC); //GTCCR set to make sure both timers are synced
    // WGM20 is used for Phase Correct PWM, COM2A1/COM2B1 sets output to non-inverted
    TCCR1A = 0;
    TCCR1A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    // PWM frequency is 16MHz/255/2/<prescaler>, prescaler is 1 here by using CS20
    TCCR1B = 0;
    TCCR1B = _BV(CS20);
 //------------------SETTING UP SECOND TIMER FOR PWM---------------------------------------------------------------
    // WGM20 is used for Phase Correct PWM, COM2A1/COM2B1/COM2A0/COM2B0 sets output to inverted
    TCCR0A = 0;
    TCCR0A = _BV(COM2A1)| _BV(COM2B1)|_BV(WGM20);
    // PWM frequency is 16MHz/255/2/<prescaler>, prescaler is 1 here by using CS20
    TCCR0B = 0;
    TCCR0B = _BV(CS20);
    GTCCR = 0; //start both timers now
//--------------------SETTING UP I/O PINS---------------------------------------------------------------------------
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(hallSensorPin1, INPUT);
    pinMode(hallSensorPin0, INPUT);
}

ISR(TIMER4_COMPA_vect)
{
  controlLoop();
}

void controlLoop()
{
    //--------------SINE WAVE CONFIGURATION------------------------------------------------------------
    //count = count+0.0003;
    //gTargetValue1 = 60*sin(2*PI*count)+290;
    //gTargetValue0 = -60*sin(2*PI*count)+290;
    //--------------READING HALL EFFECT ANALOG INPUT PINS-----------------------------------------------
    gNextSensorReadout1 = analogRead(hallSensorPin1);
    gNextSensorReadout0 = analogRead(hallSensorPin0);
    //--------------CALCULATING ERRORS-------------------------------------------------------------------
    int error1 = gTargetValue1 - gNextSensorReadout1; //Difference between current and expected values (for proportional term)
    int error0 = gTargetValue0 - gNextSensorReadout0; //Difference between current and expected values (for proportional term)
    int dError1 = (gNextSensorReadout1 - gLastSensorReadout1); 
    int dError0 = (gNextSensorReadout0 - gLastSensorReadout0);
    gLastSensorReadout1 = gNextSensorReadout1;
    gLastSensorReadout0 = gNextSensorReadout0;
    //----------------ADJUSTING DUTY CYCLE IF NECESSARY-------------------------------------------------
    if(error1 > THRESHOLDP || error1 < THRESHOLDN)
    {     
      gCurrentDutyCycle1 = gMidpoint - roundValue((gKp*error1) - (gKd*dError1));
      gCurrentDutyCycle1 = constrain(gCurrentDutyCycle1, MIN_PWM_VALUE, MAX_PWM_VALUE);
    }
    if(error0 > THRESHOLDP || error0 < THRESHOLDN)
    {
      gCurrentDutyCycle0 = gMidpoint - roundValue((gKp*error0) - (gKd*dError0));
      gCurrentDutyCycle0 = constrain(gCurrentDutyCycle0, MIN_PWM_VALUE, MAX_PWM_VALUE);
    }
    writeCoilPWM(gCurrentDutyCycle1,gCurrentDutyCycle0);
}

void loop()
{
}
