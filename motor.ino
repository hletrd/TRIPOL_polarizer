#define MAXSPEED_RPM 15 //Max speed as rpm

#define SPEEDFACTOR 1 //step per single timer event
#define MINSPEED 1 //minimum speed (steps/sec)
#define ACCRATE 500 //Acceleration rate (steps/sec^2)
#define DACCRATE 500 //Deacceleration rate (steps/sec^2)

#define OPTICALOFF 400 //Max threshold for optical encoder
#define OPTICALON 100 //Min threshold for optical encoder

#define MICROSTEP 16 //microstep drived
#define GEARING1 45
#define GEARING2 120
#define STEPS 200 //motor steps per single rev

#define PINDIR 4 //Pin for direction
#define PINSTEP 3 //Pin for step
#define PINENCODER 6 //Pin for encoder

#define USERAWPORT FALSE
#define STEPHIGH B00001000
#define STEPLOW B00000000

//Motor microstep control pins
#define pinM0 A1
#define pinM1 12
#define pinM2 13

//Motor current control pins
#define pinMCUR 11

//Motor current as mA
#define currentmA 3000

#include <Wire.h>
#include <U8x8lib.h>

#include <TimerOne.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

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

void u8drawstring(int x, int y, char* str) {
  u8x8.draw2x2String(x, y, str);
}

void u8drawstring(int x, int y, String str) {
  char tmp[20] = {0,};
  str.toCharArray(tmp, str.length()+1);
  u8x8.draw2x2String(x, y, tmp);
}

void setup() {
  Serial.begin(115200);

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_artossans8_r);
  u8x8.setContrast(50);

  u8drawstring(0, 0, "STEP");
  u8drawstring(0, 4, "ANGLE");
  
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
  
  encoder[0] = analogRead(PINENCODER);
  encoder[1] = analogRead(PINENCODER);
  encoder[2] = analogRead(PINENCODER);

  timenow = millis();
  timeold_enc = millis();
  timeold_log = millis();

  pinMode(PINDIR, OUTPUT);
  pinMode(PINSTEP, OUTPUT);

  pinMode(pinM0, OUTPUT);
  pinMode(pinM1, OUTPUT);
  pinMode(pinM2, OUTPUT);

  digitalWrite(PINDIR, HIGH);
  digitalWrite(pinM0, LOW);
  digitalWrite(pinM1, LOW);
  digitalWrite(pinM2, HIGH);

  //pinMode(pinMCUR, OUTPUT);
  analogWrite(pinMCUR, (int)(currentmA/1000.*256./5./2.));

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
    
    delayMicroseconds(100);
    while (Serial.available() > 0) {
      tmp = Serial.read();
      if (digit > 0 && tmp != '.') sum *= 10;
      if (tmp < 58 && tmp >= 48) {
        sum += (tmp - 48);
      } else if (tmp == '.') {
        break;
      }
      digit++;
      delayMicroseconds(100);
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
    String tmp = String(relpos3);
    u8drawstring(0, 2, tmp);
    tmp = String(angpos);
    u8drawstring(0, 6, tmp);
    
    
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
    encoder[2] = (analogRead(PINENCODER)+analogRead(PINENCODER))/2;

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
      encoder[2] = (analogRead(PINENCODER)+analogRead(PINENCODER))/2;
      
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
