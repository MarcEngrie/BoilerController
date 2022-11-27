//----------------------------------------------------------------------------
// defines
//----------------------------------------------------------------------------
//#define CALIBRATE1                         // calibrate first using pot-trimmer for max swing
//#define CALIBRATE2                         // calibrate ADCvolt in code

#define ADCvolt           (float)0.210     // volt per ADC count

#define ZMPT101B_GPIO     35               // define the used ADC input channel
#define CLOCKSPEED        80               // clockspeed of ESP32 in MHz
#define nomFREQ           50.00            // nominal frequency

//----------------------------------------------------------------------------
// constants
//----------------------------------------------------------------------------
// number of samples to take before RMS is calculate
const int iADC = 2 * nomFREQ;    // one sample every 1 ms -> 100ms = 5 periods @ 50Hz
// number of Vrms to hold before average Vrms is outputted
const int iRMS =  50;            // 100 x 5 periods -> 100 * 100 ms -> 5 seconds

// counters for array index
int  cntADC = 0;
int  cntRMS = 0;

// vars related to ADC and Currents
float ADCval;
float ADCzero   = 1700;

unsigned long valZER[iADC];
unsigned long avgZER[iRMS];
float         valADC[iADC];
float         valRMS[iRMS];

// needed for timer interrupt
volatile byte cntrINT;
hw_timer_t * timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//............................................................................
//** Timer interrupt
//****************************************************************************
void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  cntrINT++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//****************************************************************************

//============================================================================
// Setup
//============================================================================
void setup()
{

  Serial.begin(115200);
  delay(1000);

  #if defined(CALIBRATE1)
  
    pinMode(ZMPT101B_GPIO, INPUT);
    
  #else

    //--------------------------------------------------------------------------
    // Startup timer interrupt
    //--------------------------------------------------------------------------
    //set prescaler every microsecond eg: clockspeed 80.000.000Hz / 80 = 1.000.000Hz -> 1 µs period
    timer = timerBegin(0, CLOCKSPEED, true);
    timerAttachInterrupt(timer, &onTimer, true);
    // interrupt every 1 millisecond = 1000 µs
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    
  #endif

}
//============================================================================

//############################################################################
// Main loop
//############################################################################
void loop()
{

  #if defined(CALIBRATE1)
  
    // use Serial Plotter to view waveform
    Serial.println(analogRead(ZMPT101B_GPIO));

  #else

    float        sum    = 0;
    float        rmsVal = 0;
    unsigned int avgZer = 0;

    // interrupty occured if cntrINT > 1
    if (cntrINT > 0)
    {
      // read the ADC and save value
      valZER[cntADC] = (unsigned int)(analogRead(ZMPT101B_GPIO));
      ADCval         = (float)(valZER[cntADC]);
      ADCval         = (ADCval - ADCzero) * ADCvolt;
      valADC[cntADC] = (float)(ADCval);

      cntADC++;
      // if array is full, calc rms
      if( cntADC >= iADC)
      {
        // reset counter
        cntADC = 0;

        // find DC offset
        // skip first as it might not be correct due to long loop
        sum = 0;
        for (int n=1; n<iADC; n++)
        {
          sum = sum + (float)valZER[n];
        }
        avgZER[cntRMS] = (unsigned int)(sum / (iADC - 1));
        // adjust Zero offset for current
        ADCzero = (float)avgZER[cntRMS];

        // find RMS
        // RMS = the square root of the mean square (the arithmetic mean of the squares) of the set
        float valV;
        sum = 0;
        // skip first as it might not be correct due to long loop
        for (int n=1; n<iADC; n++)
        {
          valV = (float)valADC[n];
          sum = sum + (float)(valV * valV);
        }
        valRMS[cntRMS] = (float)(sqrt(sum / (iADC - 1)));

        cntRMS++;
        // array filled? If so, average
        if(cntRMS >= iRMS)
        {
          // reset counter
          cntRMS = 0;

          #if defined(CALIBRATE2)
            sum = 0;
            // find average DC offset
            for (int n=0; n<iRMS; n++)
            {
              sum = sum + (float)avgZER[n];
            }
            avgZer = (unsigned int)(sum / iRMS);
          #endif
          
          // find average RMS
          sum = 0;
          for (int n=0; n<iRMS; n++)
          {
            sum = sum +(float)valRMS[n];
          }
          // The average RMS value of ADC values
          rmsVal = (float)(sum / iRMS);

          Serial.printf(" Line Voltage   : %4.0f Vrms\n", rmsVal);
          #if defined(CALIBRATE2)
            Serial.printf(" Zero ADC value : %u \n\n", avgZer);
          #endif
        }
      }
      // to make sure we do not handle pilled up interrupts
      // and keep a pace of 1 every ms
      portENTER_CRITICAL(&timerMux);
      cntrINT = 0;
      portEXIT_CRITICAL(&timerMux);

    }
   #endif
}
//############################################################################
