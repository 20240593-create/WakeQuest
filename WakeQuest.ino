/* WakeQuest TWO BUTTONS - UNO R3 16MHz only.
 * B1 D1->switch->GND; B2 D10->switch->GND. INPUT_PULLUP, no button resistors.
 * D1: button1 input AFTER setup disables UART TX. Bluetooth TXD -> D0 RX.
 * Disconnect D0 and D1 wires during upload. Disconnect BT D0 for USB tests.
 * Never hold button1 during reset/boot. No Serial.print/write/flush allowed.
 * USB monitor 9600 baud, New Line: T10 timer, B buzzer 3sec. LCD diagnostics only.
 * Display each segment has 2k; active buzzer MUST use BC547 driver.
 */
#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "Select Arduino Uno (ATmega328P, 16 MHz). Not UNO R4."
#endif

const uint8_t buttonPins[2]={1,10};
const uint8_t MULTI_KEY=255;
bool buzzerTesting=false; uint32_t buzzerTestAt=0;
const uint8_t segPins[7]={2,3,4,5,6,7,8}; // a b c d e f g
const uint8_t digitPins[4]={9,11,12,13};
const uint8_t TRIG=A1, ECHO=A2, BUZZER=A3;
const uint32_t HOLD_MS=7000UL, GAP_MS=500UL, ANSWER_MS=7000UL;
const uint16_t NEAR_CM=7, FAR_CM=100;
const uint32_t MAX_SECONDS=86400UL;
const uint8_t masks[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
volatile uint8_t displayDigits[4]={0,0,0,0}; // 10 = blank

// Timer2 refresh continues while LCD/ultrasonic code is running.
// Do NOT use tone(), analogWrite(D3/D11), or libraries that own Timer2.
ISR(TIMER2_COMPA_vect) {
  static uint8_t pos=0;
  for(uint8_t i=0;i<4;i++) digitalWrite(digitPins[i],LOW);
  uint8_t v=displayDigits[pos];
  uint8_t m=(v<10)?masks[v]:0;
  for(uint8_t i=0;i<7;i++) digitalWrite(segPins[i],(m&(1<<i))?LOW:HIGH);
  if(v<10) digitalWrite(digitPins[pos],HIGH);
  pos=(pos+1)&3;
}

void showNumber(uint16_t n, bool visible=true) {
  uint8_t d[4]={(uint8_t)(n/1000%10),(uint8_t)(n/100%10),(uint8_t)(n/10%10),(uint8_t)(n%10)};
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(uint8_t i=0;i<4;i++) displayDigits[i]=visible?d[i]:10;
  }
}
void startDisplay() {
  for(uint8_t i=0;i<4;i++){ digitalWrite(digitPins[i],LOW); pinMode(digitPins[i],OUTPUT); }
  for(uint8_t i=0;i<7;i++){ digitalWrite(segPins[i],HIGH); pinMode(segPins[i],OUTPUT); }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR2A=_BV(WGM21); TCCR2B=0; TCNT2=0; OCR2A=249;
    TIFR2=_BV(OCF2A); TIMSK2=_BV(OCIE2A); TCCR2B=_BV(CS22);
  }
}

// Minimal PCF8574 LCD driver: P0 RS, P1 RW, P2 EN, P3 light, P4..7 data.
// If your backpack has a different mapping, use its matching LCD library.
uint8_t lcdAddress=0;
bool responds(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }
void lcdExp(uint8_t x){ if(!lcdAddress)return; Wire.beginTransmission(lcdAddress); Wire.write((uint8_t)(x|8)); Wire.endTransmission(); }
void lcdNibble(uint8_t n,uint8_t rs){ uint8_t x=(n<<4)|rs; lcdExp(x); lcdExp(x|4); delayMicroseconds(1); lcdExp(x); delayMicroseconds(50); }
void lcdByte(uint8_t x,uint8_t rs){ lcdNibble(x>>4,rs); lcdNibble(x&15,rs); }
void lcdCommand(uint8_t x){ lcdByte(x,0); }
void startLCD(){
  Wire.begin();
  if(responds(0x27))lcdAddress=0x27;
  else if(responds(0x3f))lcdAddress=0x3f;
  else {
    for(uint8_t a=0x20;a<=0x27 && !lcdAddress;a++)if(responds(a))lcdAddress=a;
    for(uint8_t a=0x38;a<=0x3f && !lcdAddress;a++)if(responds(a))lcdAddress=a;
  }
  if(!lcdAddress)return;
  delay(50); // LCD power-on initialization only; not a game wait.
  lcdNibble(3,0); delay(5); lcdNibble(3,0); delayMicroseconds(150);
  lcdNibble(3,0); lcdNibble(2,0);
  lcdCommand(0x28); lcdCommand(0x0c); lcdCommand(0x06); lcdCommand(0x01); delay(2);
}
void lcdLine(uint8_t row,const char* text){
  static char previous[2][17]={{0},{0}};
  char next[17]; memset(next,' ',16); next[16]=0;
  for(uint8_t i=0;i<16 && text[i];i++)next[i]=text[i];
  if(memcmp(previous[row],next,16)==0)return;
  memcpy(previous[row],next,17); lcdCommand(row?0xc0:0x80);
  for(uint8_t i=0;i<16;i++)lcdByte(next[i],1);
}

enum Stage:uint8_t { IDLE, COUNTDOWN, APPROACH, MEMORY_SHOW, MEMORY_INPUT, FEEDBACK, DONE };
void transition(Stage next,uint32_t now);
Stage stage=IDLE;
uint32_t stageAt=0, timerAt=0, durationMs=0;
uint32_t lastSensorAt=0, lastNearAt=0, holdAt=0, lastUIAt=0;
bool holding=false, feedbackGood=false;
uint16_t distanceCm=0;
uint8_t roundIndex=0, sequence[5], entered[5], inputPos=0;
uint32_t rng=0x71b29a31UL;
void transition(Stage next,uint32_t now){stage=next;stageAt=now;lastUIAt=now-200;}
uint32_t nextRandom(){ rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
void beginRound(uint32_t now){
  uint8_t len=roundIndex+3;
  for(uint8_t i=0;i<len;i++)sequence[i]=1+(nextRandom()%2);
  memset(entered,0,sizeof(entered)); inputPos=0;
  transition(MEMORY_SHOW,now);
}
bool alarmActive(){ return stage>=APPROACH && stage<=FEEDBACK; }
uint32_t remainingSeconds(uint32_t now){
  uint32_t elapsed=now-timerAt;
  return elapsed>=durationMs?0:(durationMs-elapsed+999UL)/1000UL;
}

// Digital pull-ups: released HIGH, pressed LOW; release all to rearm.
uint8_t readButtons(){
  uint8_t pressed=0,count=0;
  for(uint8_t i=0;i<2;i++)if(digitalRead(buttonPins[i])==LOW){pressed=i+1;count++;}
  return count==0?0:(count==1?pressed:MULTI_KEY); // invalid simultaneous press
}
uint8_t buttonEvent(uint32_t now){
  static uint8_t candidate=MULTI_KEY,stable=MULTI_KEY;
  static uint32_t changedAt=0;
  static bool armed=false;
  uint8_t raw=readButtons();
  if(raw!=candidate){candidate=raw;changedAt=now;}
  if(now-changedAt<30)return 0;
  stable=candidate;
  if(stable==0)armed=true;
  else if(stable==MULTI_KEY)armed=false;
  else if(armed){armed=false;return stable;}
  return 0;
}
void sampleDistance(uint32_t now){
  if(stage!=APPROACH || now-lastSensorAt<70)return;
  lastSensorAt=now;
  digitalWrite(TRIG,LOW);delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);delayMicroseconds(10);digitalWrite(TRIG,LOW);
  unsigned long pulse=pulseIn(ECHO,HIGH,8000UL); // bounded wait, <=8ms
  distanceCm=pulse?(uint16_t)(pulse/58UL):0;
  uint32_t t=millis(); // include the sensor wait
  bool near=distanceCm>=NEAR_CM && distanceCm<=FAR_CM;
  if(near){
    lastNearAt=t;
    if(!holding){holding=true;holdAt=t;}
  }else if(holding && t-lastNearAt>=GAP_MS){holding=false;}
  // A currently valid reading is required to enter the game.
  if(near && holding && t-holdAt>=HOLD_MS){roundIndex=0;beginRound(t);}
}

void command(char* text){
  if(!strcmp(text,"B")){
    buzzerTesting=true;buzzerTestAt=millis();lastUIAt=millis()-200;
    return;
  }
  if(stage!=IDLE && stage!=DONE)return;
  const char* p=text;if(*p=='T')p++;
  if(!*p)return;
  uint32_t seconds=0;
  for(;*p;p++){
    if(*p<'0'||*p>'9')return;
    seconds=seconds*10UL+(*p-'0');
    if(seconds>MAX_SECONDS)return;
  }
  if(!seconds)return;
  timerAt=millis();
  durationMs=seconds*1000UL;
  rng^=micros();if(!rng)rng=1;
  transition(COUNTDOWN,timerAt);
}
char rxBuffer[25];uint8_t rxLength=0;bool rxOverflow=false;
void receiveChar(char c){
  if(c=='\r'||c=='\n'){
    if(rxLength && !rxOverflow){rxBuffer[rxLength]=0;command(rxBuffer);}
    rxLength=0;rxOverflow=false;return;
  }
  if(c<32||c>126)return;
  if(rxLength<sizeof(rxBuffer)-1)rxBuffer[rxLength++]=c;
  else rxOverflow=true;
}
void updateGame(uint32_t now,uint8_t key){
  if(stage==COUNTDOWN && !remainingSeconds(now)){
    holding=false;distanceCm=0;transition(APPROACH,now);
  }else if(stage==MEMORY_SHOW){
    if(now-stageAt>=(roundIndex==2?4000UL:3000UL))transition(MEMORY_INPUT,now);
  }else if(stage==MEMORY_INPUT){
    if(now-stageAt>=ANSWER_MS){feedbackGood=false;transition(FEEDBACK,now);}
    else if(key){
      entered[inputPos]=key;
      if(key!=sequence[inputPos]){feedbackGood=false;transition(FEEDBACK,now);}
      else if(++inputPos==roundIndex+3){feedbackGood=true;transition(FEEDBACK,now);}
    }
  }else if(stage==FEEDBACK && now-stageAt>=1000UL){
    if(feedbackGood && ++roundIndex==3)transition(DONE,now);
    else beginRound(now);
  }
}
void updateUI(uint32_t now){
  if(now-lastUIAt<100)return;
  lastUIAt=now;
  char first[17]={0},second[17]={0}; uint16_t number=0; bool visible=true;
  switch(stage){
    case IDLE:
      strcpy(first,buzzerTesting?"BUZZER TEST 3s":"2BTN D1 D10");
      snprintf(second,sizeof(second),"B1:%u B2:%u",digitalRead(1)==LOW?1:0,digitalRead(10)==LOW?1:0);
      break;
    case COUNTDOWN:{
      uint32_t s=remainingSeconds(now);
      snprintf(first,sizeof(first),"LEFT %02lu:%02lu:%02lu",s/3600UL%100UL,s/60UL%60UL,s%60UL);
      if(s>=3600UL){number=(s/3600UL)*100UL+(s/60UL%60UL);strcpy(second,"7SEG: HHMM");}
      else {number=(s/60UL)*100UL+s%60UL;strcpy(second,"7SEG: MMSS");}break;
    }
    case APPROACH:{
      uint32_t elapsed=holding?now-holdAt:0;
      uint16_t left=elapsed>=HOLD_MS?0:(HOLD_MS-elapsed+999UL)/1000UL;
      number=left;
      snprintf(first,sizeof(first),"STAY NEAR %02us",left);
      if(distanceCm)snprintf(second,sizeof(second),"D:%3ucm 7-100",distanceCm);
      else strcpy(second,"NO VALID ECHO");
      if(!holding)visible=(now/400UL)%2==0;
      break;
    }
    case MEMORY_SHOW:
    case MEMORY_INPUT:{
      number=roundIndex+1;
      snprintf(first,sizeof(first),"ROUND %u/3 %s",roundIndex+1,stage==MEMORY_SHOW?"LOOK":"INPUT");
      uint8_t len=roundIndex+3;
      for(uint8_t i=0;i<len;i++){
        uint8_t v=(stage==MEMORY_SHOW)?sequence[i]:entered[i];
        second[2*i]=v?('0'+v):'_';second[2*i+1]=' ';
      }
      second[2*len]=0;break;
    }
    case FEEDBACK:number=roundIndex+1;strcpy(first,feedbackGood?"CORRECT!":"WRONG / TIMEOUT");strcpy(second,feedbackGood?"NEXT...":"RETRY THIS ROUND");break;
    case DONE:number=8888;strcpy(first,"WAKE UP SUCCESS!");strcpy(second,"ALARM OFF");break;
  }
  showNumber(number,visible);lcdLine(0,first);lcdLine(1,second);
}
void setup(){
  digitalWrite(BUZZER,LOW);pinMode(BUZZER,OUTPUT);
  digitalWrite(TRIG,LOW);pinMode(TRIG,OUTPUT);pinMode(ECHO,INPUT);
  Serial.begin(9600);
  UCSR0B &= (uint8_t)~_BV(TXEN0); // release D1 from UART TX before enabling pull-up
  for(uint8_t i=0;i<2;i++)pinMode(buttonPins[i],INPUT_PULLUP);
  startDisplay();startLCD();
}
void loop(){
  while(Serial.available())receiveChar((char)Serial.read());
  uint32_t now=millis();uint8_t key=buttonEvent(now);
  sampleDistance(now);now=millis();
  updateGame(now,key);
  if(buzzerTesting && now-buzzerTestAt>=3000UL)buzzerTesting=false;
  digitalWrite(BUZZER,(buzzerTesting || (alarmActive() && ((now-stageAt)%1500UL<150UL)))?HIGH:LOW);
  updateUI(now);
}
