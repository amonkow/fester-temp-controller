
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
#define AC_CONTROL_INTERVAL 10

// Interval of calculating new PID parameters
// CONSIDER TIME CONSTANTS OF SENSORS WHEN ADJUSTING
#define PID_INTERVAL 100000
#define TEMP_R_REF 10000

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
uint NTC805R[] = {175971, 142862, 116641, 95761, 79023, 65536, 54608, 45709, 38425, 32437, 27491, 23388, 19972, 17115, 14716, 12696, 10988, 9538, 8305, 7252, 6349, 5574, 4907, 4330, 3830, 3396, 3018, 2688, 2400, 2146, 1924};

DAT T1 = {A0, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T2 = {A1, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T3 = {A6, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T4 = {A7, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T5 = {A8, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT T6 = {A9, 0, 0, 0, NTC805R, NELEMS(NTC805R), 0, 150000, 5000};
DAT A_CHAN[] = {T1, T2, T3, T4, T5, T6};
#define NCHANS 6

struct TIME {
  uint tPrev;
  uint tInt;
};

#define AC_SENSE 1
#define ANALOG_READ 2
#define PID_CALC 3

TIME T_AC_SENSE = {0, AC_CONTROL_INTERVAL};
TIME T_ANALOG_READ = {0, ANALOG_TIME_CONSTANT / (1<<ANALOG_TEMP_SMOOTHING)};
TIME T_PID_CALC = {0, PID_INTERVAL};
int current_time;

struct PID {
  int Kp;
  int Ki;
  int Kd;
  int interr;
  int prev_err;
};

boolean timeout_expired(int which) {
  switch (which) {
    case AC_SENSE:
      if (current_time - T_AC_SENSE.tPrev >= T_AC_SENSE.tInt) {
        T_AC_SENSE.tPrev = current_time;
        return true;
      }

    case ANALOG_READ:
      if (current_time - T_ANALOG_READ.tPrev >= T_ANALOG_READ.tInt) {
        T_ANALOG_READ.tPrev = current_time;
        return true;
      }

    case PID_CALC:
      if (current_time - T_PID_CALC.tPrev >= T_PID_CALC.tInt) {
        T_PID_CALC.tPrev = current_time;
        return true;
      }
  }
  return false; 
}

void setup() {
  // put your setup code here, to run once:
  pinMode(TE_PWM, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(PWR_OK, INPUT);
  pinMode(TE_POL_OUT, OUTPUT);
  pinMode(AC_CONTROL, OUTPUT);
  pinMode(TE_PWM_IN, INPUT);
  pinMode(AC_LINE_SENSE, INPUT);
  
  analogReference(DEFAULT);
  analogReadRes(16);
  for (int i=0; i<NCHANS; i++) {
    A_CHAN[i].value = analogRead(A_CHAN[i].pin) << 16;
  }
  
  Serial.begin(9600);
  current_time = micros();
}

void loop() {
  // Only run individual tasks on appropriate individual 
  // roll-over safe?
  current_time = micros();

  // Analog read task
  if (timeout_expired(ANALOG_READ)) {
    updateAnalog();
  }

  // AC control task
  if (timeout_expired(AC_SENSE)) {
    ac_line_sense();
  }

  // PID calculation task
  if (timeout_expired(PID_CALC)) {
    calcTemps();
    for (int i=0; i<NCHANS; i++) {
      Serial.print("Analog Channel ");
      Serial.print(A_CHAN[i].pin);
      Serial.print(": Count=");
      Serial.print(A_CHAN[i].value >> 16); 
      Serial.print(" : R=");
      Serial.print(A_CHAN[i].r*TEMP_R_REF>>16);
      Serial.print(" : T=");
      Serial.println(float(A_CHAN[i].temp)/1000);
    }

    while (Serial.available()) {
    }
      
    analogWrite(TE_PWM, 216);
    analogWrite(FAN_PWM, 255);
    digitalWrite(TE_POL_OUT, 1);
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

void ac_line_sense() {
  int i = 0;
}

