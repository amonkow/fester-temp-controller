
// PIN DEFINITIONS

#define TE_PWM 3
#define FAN_PWM 4
#define PWR_OK 5
#define AC_LINE_SENSE 6
#define TE_POL_OUT 7
#define AC_CONTROL 9
#define TE_PWM_IN 16

// Invervals in microseconds
// AC CONTROL must be fast enough to detect 120Hz cycle edge
#define AC_CONTROL_INTERVAL 1000
// Interval of calculating new PID parameters
// CONSIDER TIME CONSTANTS OF SENSORS WHEN ADJUSTING
#define PID_INTERVAL 1000000
#define TEMP_R_REF 10000
#define MICROS_PER_SECOND 1000000

// Digital filter timeconstant in microseconds
#define ANALOG_TIME_CONSTANT 2000000

// Number of calculations to smooth, 5 is probably sufficient 5=1<<5 readings / time-constant period
#define ANALOG_TEMP_SMOOTHING 5

//MISC
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

struct DAT {
  byte pin;
  uint value;
  uint r;
  int temp;
  uint *tmap;
  int maplen;
  int tmin;
  int tmax;
  int tinc;
};

// Resistance values in for tmin to tmax in tinc increments
// Units are R * 1<<16/Rref
// Temp units arbitrary, suggest C*1e3
uint NTC805R[] = {175971, 142862, 116641, 95761, 79023,
                  65536, 54608, 45709, 38425, 32437,
                  27491, 23388, 19972, 17115, 14716,
                  12696, 10988, 9538, 8305, 7252,
                  6349, 5574, 4907, 4330, 3830,
                  3396, 3018, 2688, 2400, 2146, 1924};

DAT T1 = {A0, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T2 = {A1, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T3 = {A6, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T4 = {A7, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T5 = {A8, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T6 = {A9, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T7 = {A10, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T8 = {A11, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT A_CHAN[] = {T1, T2, T3, T4, T5, T6, T7, T8};
#define NCHANS 8

struct TIME {
  uint tPrev;
  uint tInt;
};

TIME T_AC_SENSE = {0, AC_CONTROL_INTERVAL};
TIME T_ANALOG_READ = {0, ANALOG_TIME_CONSTANT / (1<<ANALOG_TEMP_SMOOTHING)};
TIME T_PID_CALC = {0, PID_INTERVAL};
TIME T_AC_CYCLE_MIN = {0, 5000}; // 5 ms min detection time
TIME T_AC_CYCLE_TIMEOUT = {0, 100000}; // 100 ms max detection time

struct PID {
  long target; //C * 1000
  unsigned long tPrev;
  long Kp; // 1/C
  long Ki; // 1/hr-C
  long Kd; // C/min
  long Perr;
  long Derr;
  long Ierr;
  long prev_err;
  long signal_out;
};

PID TE_PID = {
  .target = 20000,
  .tPrev = 0,
  .Kp = 10,
  .Ki = 30,
  .Kd = 0,
  .Perr = 0,
  .Derr = 0,
  .Ierr = 0,
  .prev_err = 0,
  .signal_out = 0,
};

PID AC_PID = {
  .target = 20000,
  .tPrev = 0,
  .Kp = 10,
  .Ki = 30,
  .Kd = 0,
  .Perr = 0,
  .Derr = 0,
  .Ierr = 0,
  .prev_err = 0,
  .signal_out = 0,
};


/* AC sense variables */
float count_on_average = 0;
float count_off_average = 0;
int count_on = 0;
int count_off = 0;
int count_weight = 100;
boolean count_was_high = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(TE_PWM, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(PWR_OK, INPUT);
  pinMode(TE_POL_OUT, OUTPUT);
  pinMode(AC_CONTROL, OUTPUT);
  pinMode(TE_PWM_IN, INPUT);
  pinMode(AC_LINE_SENSE, INPUT);
  
  analogWriteFrequency(TE_PWM, 32767);
  
  analogReference(DEFAULT);
  analogReadRes(16);
  for (int i=0; i<NCHANS; i++) {
    A_CHAN[i].value = analogRead(A_CHAN[i].pin) << 16;
  }
  Serial.begin(115200);
  Serial.println('Hello');
  AC_PID.tPrev = millis();
  TE_PID.tPrev = millis();
}

void loop() {
  // Only run individual tasks on appropriate individual 
  // roll-over safe?

  // Analog read task
  if (checkInterval(&T_ANALOG_READ)) {
    updateAnalog();
  }

  // AC control task & TE control task
  if (checkInterval(&T_AC_SENSE)) {
    //AC_line_sense_set(&AC_PID);
    TE_line_set(&TE_PID);
  }

  // PID calculation task
  if (checkInterval(&T_PID_CALC)) {
    calcTemps();
    update_PID(&TE_PID, &A_CHAN[2]); //Controlling the TE PID loop to Temp channel 2 (TR3)
    //update_PID(T_AC_PID, A_CHAN[0], 80);
    for (int i=0; i<3; i++) {
      Serial.print("Ch");
      Serial.print(A_CHAN[i].pin);
      Serial.print(": Cnt=");
      Serial.print(A_CHAN[i].value >> 16); 
      Serial.print(" : R=");
      Serial.print(A_CHAN[i].r*TEMP_R_REF>>16);
      Serial.print(" : T=");
      Serial.println(float(A_CHAN[i].temp)/1000);
    }
    Serial.print("P=");
    Serial.print(TE_PID.Perr);
    Serial.print(" I=");
    Serial.print(TE_PID.Ierr);
    Serial.print(" D=");
    Serial.print(TE_PID.Derr);
    Serial.print(" Set=");
    Serial.print(TE_PID.target);
    Serial.print(" Err=");
    Serial.print(TE_PID.prev_err);
    Serial.print(" Sig=");
    Serial.println(TE_PID.signal_out);

    while (Serial.available()) {
      int read_num = Serial.parseInt();
      Serial.print("Received ");
      Serial.println(read_num);
      if ((read_num > 1000) & (read_num < 50000)) {
        TE_PID.target = read_num;
        Serial.print("Set Temperature to: ");
        Serial.println(read_num/1000);
      }
    }
  }
}

bool checkInterval(struct TIME *task) {
  unsigned long current_time = micros();
  if(current_time - task->tPrev >= task->tInt) {
    task->tPrev = current_time;
    return true;
  } else {
    return false;
  }
}

void updateAnalog() {
  //Stores value in a full 32 bit uint to accommodate smoothing intiger math
  for (int i=0; i<NCHANS; i++) {
    uint reading = analogRead(A_CHAN[i].pin) << 16;
    int diff = int(reading - A_CHAN[i].value) >> ANALOG_TEMP_SMOOTHING;
    A_CHAN[i].value += diff;
  }
}

void calcTemps() {
  for (int i=0; i<NCHANS; i++) {
    //Calculate non-dimensionalized resistance value (*1<<16/Rref)
    A_CHAN[i].r = (( A_CHAN[i].value / ( (1<<16) - (A_CHAN[i].value>>16) ) ));
    //If value is off map, cap temperature value
    if (A_CHAN[i].r > A_CHAN[i].tmap[0]) {
      A_CHAN[i].temp = A_CHAN[i].tmin;
    } else if (A_CHAN[i].r < A_CHAN[i].tmap[A_CHAN[i].maplen-1]) {
      A_CHAN[i].temp = A_CHAN[i].tmax;
    //Otherwise, calculate lookup value after finding index
    } else {
      int idx = 1;
      for  (idx=1; idx<A_CHAN[i].maplen; idx++) {
        if (A_CHAN[i].tmap[idx] < A_CHAN[i].r) {
          break;
        }
      }
      A_CHAN[i].temp = map(A_CHAN[i].r,
                           A_CHAN[i].tmap[idx-1],
                           A_CHAN[i].tmap[idx],
                           A_CHAN[i].tinc*(idx-1) + A_CHAN[i].tmin,
                           A_CHAN[i].tinc*(idx) + A_CHAN[i].tmin);
    }
  }
}

void TE_line_set(struct PID *PID_struct) {
  int polarity = PID_struct->signal_out >= 0;
  int abs_pwr = abs(PID_struct->signal_out);
  
  digitalWrite(TE_POL_OUT, polarity);
  analogWrite(TE_PWM, abs_pwr);
  analogWrite(FAN_PWM, abs_pwr);
}

void AC_line_sense_set(struct PID *PID_struct) {
  long current_time = micros();
  static uint sense_state = 0;
  int cur_sense_state = digitalRead(AC_LINE_SENSE);

  if (sense_state == 1 && cur_sense_state == 0) {
    // Stepdown detected, check timing
    if (current_time - T_AC_CYCLE_MIN.tPrev >= T_AC_CYCLE_MIN.tInt) {
      update_line_state(PID_struct->signal_out);
      T_AC_CYCLE_MIN.tPrev = current_time;
      T_AC_CYCLE_TIMEOUT.tPrev = current_time;
    } else {
      update_line_state(0);
      // IN ERROR STATE, TOO MANY EDGES DETECTED
      T_AC_CYCLE_MIN.tPrev = current_time;
      T_AC_CYCLE_TIMEOUT.tPrev = current_time;
    }
    sense_state = cur_sense_state;
  }
  if (current_time - T_AC_CYCLE_TIMEOUT.tPrev >= T_AC_CYCLE_TIMEOUT.tInt) {
    //IN ERROR STATE - NO LINE CHANGES DETECTED
    update_line_state(0);
    //do not update tPrev for this timeout state
  }
}

void update_line_state(int PID_pwr_setting) {
  //Update power state based on current power target from PID calc if detect step down
  
  //scaling for safe intiger division math
  uint scale = 8;
  //bits of cycling, 8=256 cycles ~= 2 sec @ 120cyc/sec, 60Hz
  //sets how many cycles will pass for each power pulse for power_setting = 1
  //also defines resolution of power control and maximum useful pwr_setting.
  uint cycles = 8;
  uint rollover = 32 - 1 - scale; // bits to do forced rollover - need both counters to roll simultaniously
  static uint cycle_int = 0;
  static uint pwr_int = 0;
  //If the power setting is 0 or lower, shut all power off
  if (PID_pwr_setting < 1) {
    digitalWrite(AC_CONTROL, 0);
    cycle_int = 0;
    pwr_int = 0;
  } else {
    /*
    In this case, we are keeping 'racing' counters, we update
    the pwr_int with at a weighted rate inverse to the pwr_setting
    only incrementing when power is on, and a cycle_int counter 
    that updates linearly regardless of power state.
    When the pwr_setting is maximum, the pwr_int updates at the 
    same rate as the cycle_int, so power always stays on.
    The problem with this algroithm is that there is an effective
    response delay of 2^cycles/pwr_setting. So if power setting is
    1, we will not respond to increased power for 2^cycles
    (~2 seconds for 256 cycles @ 120 cycles per second i.e. 60 Hz)
    */
    if (pwr_int < cycle_int<<scale) {
      digitalWrite(AC_CONTROL, 1);
      int pwr_multiplier = ((1<<cycles)<<scale) / PID_pwr_setting;
      pwr_int += (pwr_multiplier);
    } else {
      digitalWrite(AC_CONTROL, 0);
    }
    cycle_int++;
    //Force simultanious rollover
    uint cycle_max = 1<<rollover;
    if (cycle_int >= cycle_max && pwr_int >= cycle_max) {
      cycle_int %= cycle_max;
      pwr_int %= cycle_max;
    }
  }
}

void update_PID(struct PID *PID, struct DAT *T) {
  // UNITS!!! - controlable signal_out is only: -256 to 256 (for TE), 0 to 256 (for AC)
  // Kp = pwr/C
  unsigned long current_time = millis();

  //calculate error
  long err = T->temp - PID->target; // DegC * 1000
  
  // calculate dt
  long dt = current_time - PID->tPrev; //microseconds
  PID->tPrev = current_time;
  
  //calculate error components
  PID->Perr = (err * PID->Kp);
  PID->Ierr += (err * PID->Ki * dt) / 3600000; //dt is going to be ~ 100
  PID->Derr = ((err - PID->prev_err) * PID->Kd * 60 / dt) * 1000;
  
  //calcualte singnal out
  PID->signal_out = (PID->Perr + PID->Ierr + PID->Derr) / 1000;
  
  //store error for next cycle
  PID->prev_err = err;
}
