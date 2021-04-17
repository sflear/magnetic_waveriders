#define MIN_PWM_VALUE                   0        //Minimum PWM duty cycle
#define MAX_PWM_VALUE                   255      //Maximum PWM duty cycle 
#define DEFAULT_KP                      120    
#define DEFAULT_KD                      80
#define PROPORTIONAL_THRESHOLD          100

int roundValue(float value)
{
  return (int)(value + 0.5);
}
int PROP_THRESHOLDP = PROPORTIONAL_THRESHOLD;
int PROP_THRESHOLDN = -1*PROPORTIONAL_THRESHOLD;
float count = 0;
const int hallSensorPin1 = 1;
const int gMidpoint = roundValue((MAX_PWM_VALUE - MIN_PWM_VALUE) / 2); //The midpoint of our PWM range

int gCurrentDutyCycle1 = 0; //Current PWM duty cycle for the coil
int gLastSensorReadout1 = 0; //Last sensor readout to calculate derivative term
int gNextSensorReadout1 = 0; //The "next" sensor value. Declared global so we can use it as a running average and move ot to gLastSensorReadout after PWM calculation

int gTargetValue1 = 185;
float gKp = DEFAULT_KP;
float gKd = DEFAULT_KD;

void writeCoilPWM(int value1)
{
    OCR1B = value1; 
}

void setup()
{  
     noInterrupts();
     //set timer4 interrupt at 1Hz
     TCCR4A = 0;// set entire TCCR1A register to 0
     TCCR4B = 0;// same for TCCR1B
     TCNT4  = 0;//initialize counter value to 0
     // set compare match register for approximately 4Khz increments
     OCR4A = 50;
     // turn on CTC mode
     TCCR4B |= (1 << WGM12);
     // Set CS12 and CS10 bits for 64 prescaler
     TCCR4B |= (1 << CS11) | (1 << CS10);  
     // enable timer compare interrupt
     TIMSK4 |= (1 << OCIE4A);
    interrupts();//allow interrupts
  
    // Setup timer 1 as Phase Correct non-inverted PWM, 31372.55 Hz.
    pinMode(12, OUTPUT);
    //GTCCR = _BV(TSM) | _BV(PSRSYNC); //GTCCR set to make sure both timers are synced
    // WGM20 is used for Phase Correct PWM, COM2A1/COM2B1 sets output to non-inverted
    TCCR1A = 0;
    TCCR1A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    // PWM frequency is 16MHz/255/2/<prescaler>, prescaler is 1 here by using CS20
    TCCR1B = 0;
    TCCR1B = _BV(CS20);

    pinMode(hallSensorPin1, INPUT);
}

ISR(TIMER4_COMPA_vect)
{
  serviceRoutine();
}

void serviceRoutine()
{
    count = count+0.0004;
    gTargetValue1 = 70*sin(2*PI*count)+175;
    gNextSensorReadout1 = analogRead(hallSensorPin1);
    int error1 = gTargetValue1 - gNextSensorReadout1; //Difference between current and expected values (for proportional term)
    int dError1 = (gNextSensorReadout1 - gLastSensorReadout1); 
    gLastSensorReadout1 = gNextSensorReadout1;
    if(error1 > PROP_THRESHOLDP || error1 < PROP_THRESHOLDN)
    {     
      gCurrentDutyCycle1 = gMidpoint - roundValue((gKp*error1) - (gKd*dError1));
      gCurrentDutyCycle1 = constrain(gCurrentDutyCycle1, MIN_PWM_VALUE, MAX_PWM_VALUE);
    }
    writeCoilPWM(gCurrentDutyCycle1);
}

void loop()
{
}
