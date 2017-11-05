#define MAXSPEED_RPM 20 //as rpm
#define speedfactor 1 //microstep to go for single timer event
#define minspeed 5 //minimum speed (steps/sec)
#define ACCRATE 1500 //Acceleration rate
#define DACCRATE 1500 //Deacceleration rate

#define thmax 700 //Max threshold for optical encoder
#define thmin 200 //Min threshold for optical encoder


#define pinCCW 2 //PD1, pin for rotation
#define pinENCODER 0 //PF7

//120 / 45 gearing, 1/16 microstep, 200step/rev motor
#define stepperrev 8533.333333333333333333333333



#define userawport true 
#define CCWHIGH B00000100
#define CCWLOW B00000000


#include <TimerOne.h>

uint32_t MAXSPEED;
uint32_t interval;
float acc;
float dacc;

uint32_t posto;
uint32_t posnow;
uint32_t half;

float dacctime;
float spd, oldspd;
uint32_t daccsteps;

uint32_t timeold[2];
uint32_t timenow;

int encoder[2];

void movestepper() {
  if (posto > posnow) {
    if (posto - posnow < daccsteps && spd > minspeed && posto - posnow <= half) {
      spd -= (float)dacc * interval / 1000000.+1e-15;
      if (spd < minspeed) spd = minspeed;
      changeTimer();
    } else if (spd < MAXSPEED) {
      spd += (float)acc * interval / 1000000.+1e-15;
      if (spd > MAXSPEED) spd = MAXSPEED;
      changeTimer();
    }
    for (int i = 0; i < speedfactor; i++) {
#if useRawPort == true
      PORTD = CCWLOW;
#else
      digitalWrite(pinCCW, LOW);
#endif
      delayMicroseconds(5);
#if useRawPort == true
      PORTD = CCWHIGH;
#else
      digitalWrite(pinCCW, HIGH);
#endif
      delayMicroseconds(5);
      posnow += 1;
    }
#if useRawPort == true
    PORTD = CCWLOW;
#else
    digitalWrite(pinCCW, LOW);
#endif
  } else {
    spd = minspeed;
    changeTimer();
  }
}

inline void changeTimer() {
  if (spd != oldspd) {
    oldspd = spd;
    interval = (uint32_t) (1000000. / spd);
    Timer1.detachInterrupt();
    Timer1.attachInterrupt(movestepper, interval);
  }
}

void setup() {
  Serial.begin(115200);
  spd = minspeed;
  oldspd = 0;
  interval = (int)(1000000./minspeed);
  acc = ACCRATE;
  dacc = DACCRATE;

  posto = 0;
  posnow = 0;
  
  half = 0;

  MAXSPEED = (uint32_t) MAXSPEED_RPM * stepperrev / 60;

  pinMode(pinCCW, OUTPUT);
  
  Timer1.initialize();
  changeTimer();
}

void loop() {
  if (Serial.available() > 0) {
    float sum = 0;
    byte tmp;
    int digit = 0;
    
    delayMicroseconds(12);
    while (Serial.available() > 0) {
      tmp = Serial.read();
      if (digit > 0 && tmp != '.') sum *= 10;
      if (tmp < 58 && tmp >= 48) {
        sum += (tmp - 48);
      } else if (tmp == '.') {
        break;
      }
      digit++;
      
      delayMicroseconds(12);
    }
    digit = 1;
    while (Serial.available() > 0) {
      tmp = Serial.read();
      sum += pow(0.1, digit) * (tmp-48);
      digit++;
    }
    
    if (sum > -1e+9 && sum < 1e+9) {
      Serial.print("\n\nMove to...");
      Serial.println(sum);
      sum = (uint32_t)(sum / 360. * stepperrev);
      half = sum / 2;
      posto += sum;
    }   

    dacctime = (float)MAXSPEED / dacc;
    daccsteps = (uint32_t)(0.5 * dacc * dacctime * dacctime);
    
  }

  timenow = millis();
  
  if (timenow > timeold[0]+1000) {
    timeold[0] = timenow;
    Serial.print("\n\nSpeed now: ");
    Serial.println(spd);
    Serial.print("Step now: ");
    Serial.println(posnow);
    Serial.print("Degree now: ");
    Serial.println(((uint32_t)(posnow / stepperrev * 360*100)) % 36000 / 100.);
  }

  encoder[0] = encoder[1];
  encoder[1] = encoder[2];
  encoder[2] = (analogRead(0)+analogRead(0)) / 2;

  if (timenow > timeold[1]+50) {
    if (encoder[2] > thmax && encoder[1] > thmax && encoder[0] < thmin) {
      timeold[1] = timenow;
      Serial.println("Encoder pulse");
      posnow = 0;
    }
  }
}
