#define MAXSPEED_RPM 20 //Max speed as rpm

#define SPEEDFACTOR 1 //step per single timer event
#define MINSPEED 5 //minimum speed (steps/sec)
#define ACCRATE 2000 //Acceleration rate (steps/sec^2)
#define DACCRATE 2000 //Deacceleration rate (steps/sec^2)

#define OPTICALOFF 400 //Max threshold for optical encoder
#define OPTICALON 100 //Min threshold for optical encoder

#define MICROSTEP 16 //1/16 microstep drived
#define GEARING1 45
#define GEARING2 120
#define STEPS 200 //motor steps per single rev

#define PINSTEP 3 //PD1, pin for step
#define PINENCODER //PF7, pin for encoder

#define USERAWPORT TRUE
#define STEPHIGH B00001000
#define STEPLOW B00000000


#include <TimerOne.h>

uint32_t abspos_now; //Absolute position now (never decreases)
uint32_t abspos_to; //Absolute position desired
uint32_t relpos3; //Relative position (to encoder)
uint32_t revs3; //Revs now
uint32_t posdiff; //relpos - abspos
float angpos; //Angular position (relative to encoder)


float speed, oldspeed;
uint32_t interval; //Timer interval (1000000 / speed)
uint32_t daccsteps; //Steps to deaccelerate from full speed to zero
uint32_t half; //Variable to store half position between acc and dacc

int encoder[3] = {0, 0, 0}; //buffer to store encoder value

uint32_t stepperrev3; //step per 3 revs
uint32_t stepperrev1; //step per single rev
uint32_t maxspeed_steps; //maxspeed as steps per sec

uint32_t timenow;
uint32_t timeold_enc, timeold_log;

bool ismoving;

void setup() {
  Serial.begin(115200);

  abspos_now = 0;
  abspos_to = 0;
  relpos3 = 0;
  revs3 = 0;
  posdiff = 0;
  angpos = 0.0;

  //initialize variables
  speed = MINSPEED;
  oldspeed = 0; //have to differ from speed to trigger setTimer() initialization
  interval = (uint32_t) (1000000. / speed);
  maxspeed_steps = (uint32_t)((float)MAXSPEED_RPM * MICROSTEP * GEARING2 * STEPS / GEARING1 / 60);
  
  float dacctime = (float)maxspeed_steps / DACCRATE;
  daccsteps = (uint32_t)(0.5 * DACCRATE * dacctime * dacctime);

  half = 0;

  stepperrev3 = (uint32_t) 3 * MICROSTEP * GEARING2 * STEPS / GEARING1;
  stepperrev1 = (uint32_t) MICROSTEP * GEARING2 * STEPS / GEARING1;
  
  ismoving = false;
  
  encoder[0] = analogRead(0);
  encoder[1] = analogRead(0);
  encoder[2] = analogRead(0);

  timenow = millis();
  timeold_enc = millis();
  timeold_log = millis();
  
  
  pinMode(PINSTEP, OUTPUT);

  Serial.println("Init");
  
  Timer1.initialize();
  setTimer();

  findEncoder();
}

void loop() {
  if (Serial.available() > 0) {
    float sum = 0;
    byte tmp;
    int digit = 0;
    float angtorotate;
    
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
    
    if (sum <= 36000.0 && ismoving == false) {
      Serial.print("\n\nmoveto:");
      Serial.println(sum);
      if (angpos < sum) {
        angtorotate = sum - angpos;
      } else {
        angtorotate = 360.0 - (angpos - sum);
      }
      abspos_to += (uint32_t)(angtorotate / 360.0 / 3.0 * stepperrev3);
      half = (uint32_t)(angtorotate / 360.0 / 3.0 * stepperrev3/2);
      ismoving = true;
    } 
  }
  
  timenow = millis();
  
  if (timenow > timeold_log+200) {
    timeold_log = timenow;
    Serial.print("\n\nabspos_now:");
    Serial.println(abspos_now);
    Serial.print("abspos_to:");
    Serial.println(abspos_to);
    Serial.print("relpos3:");
    Serial.println(relpos3);
    Serial.print("revs3:");
    Serial.println(revs3);
    Serial.print("posdiff:");
    Serial.println(posdiff);
    Serial.print("angpos:");
    Serial.println(angpos);
    Serial.print("speed:");
    Serial.println(speed);
  }

  if (timenow >= timeold_enc + 1) {
    timeold_enc = timenow;
    encoder[0] = encoder[1];
    encoder[1] = encoder[2];
    encoder[2] = (analogRead(0)+analogRead(0))/2;

    if (encoder[2] < OPTICALON && encoder[1] < OPTICALON && encoder[0] > OPTICALOFF) {
      //Serial.print("Found encoder\nSync:");
      //int32_t dsyncamount = (abspos_now % stepperrev3 - posdiff) % stepperrev1;
      //Serial.println(dsyncamount);
      //if (dsyncamount > 50) abspos_to += dsyncamount;
      posdiff = abspos_now % stepperrev3;
      relpos3 = 0;
    }
  }
}


void movestepper() {
  if (abspos_to > abspos_now) {
    if (abspos_to - abspos_now < daccsteps && //Start dacc
        speed > MINSPEED && //Speed above minimum speed
        abspos_to - abspos_now <= half) { //Equal or more than half step
      speed -= (float)DACCRATE * interval / 1000000.;
      if (speed < MINSPEED) speed = MINSPEED;
      setTimer();
    } else if (speed < maxspeed_steps) {
      speed += (float)ACCRATE * interval / 1000000.;
      if (speed > maxspeed_steps) speed = maxspeed_steps;
      setTimer();
    }
    for (int i = 0; i < SPEEDFACTOR; i++) {
#if useRawPort == true
      PORTD = STEPLOW;
#else
      digitalWrite(PINSTEP, LOW);
#endif
      delayMicroseconds(5);
#if USERAWPORT == true
      PORTD = STEPHIGH;
#else
      digitalWrite(PINSTEP, HIGH);
#endif
      delayMicroseconds(5);
      abspos_now += 1;
      relpos3 += 1;
      if (relpos3 > stepperrev3) {
        relpos3 = (abspos_now - posdiff) % stepperrev3;
        revs3 = (abspos_now - posdiff) / stepperrev3;
      }

      angpos = ((uint32_t)(relpos3 * 360.0 * 3.0 / stepperrev3*1000) % 360000) / 1000.0;   
    }
#if USERAWPORT == true
    PORTD = STEPLOW;
#else
    digitalWrite(PINSTEP, LOW);
#endif
  } else {
    speed = MINSPEED;
    setTimer();
    ismoving = false;
  }
}

inline void setTimer() {
  if (speed != oldspeed) {
    oldspeed = speed;
    interval = (uint32_t) (1000000. / speed);
    Timer1.detachInterrupt();
    Timer1.attachInterrupt(movestepper, interval);
  }
}

void findEncoder() {
  abspos_to = stepperrev3;
  half = abspos_to / 2;
  ismoving = true;

  while(true) {
    timenow = millis();
    
    if (timenow >= timeold_enc+1) {
      timeold_enc = timenow;
      
      encoder[0] = encoder[1];
      encoder[1] = encoder[2];
      encoder[2] = (analogRead(0)+analogRead(0))/2;
      if (encoder[2] < OPTICALON && encoder[1] < OPTICALON && encoder[0] > OPTICALOFF) {
        Serial.println("Found encoder");
        posdiff = abspos_now;
        abspos_to = revs3 * stepperrev3 + (relpos3 / stepperrev1+1)*stepperrev1 + posdiff;
        relpos3 = 0;
        break;
      }
    }
  }
}
