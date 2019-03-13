 
#include <stdio.h> //for printf
#include <stdint.h> // for uint~~
#include <assert.h>
#include <stdbool.h>
#include <io.h>
#include <delay.h>

#define sa PORTB.0
#define sb PORTB.1
#define sc PORTB.2
#define sd PORTB.3

#define clk PORTD.5
#define data PORTD.6
#define str PORTD.7

#define DC1 PORTB.4
#define DC2 PORTB.5

//#define SEN1 PINB.6
//#define SEN2 PINB.7
#define SEN1 PINB.7
#define SEN2 PINB.6
#define PB1  PINA.1
#define PB2  PINA.2
#define PB3  PINA.3
#define PB4  PINA.4
#define BZ   PORTA.0
#define ena  PINA.5   
#define enb  PINA.6   
#define ensw PINA.7   

#define COM5 PORTD.4 
#define COM1 PORTD.0
#define COM2 PORTC.7
#define COM3 PORTD.2
#define COM4 PORTD.3
  
#define DATA7 PORTC.6
#define DATA6 PORTC.5
#define DATA5 PORTC.4
#define DATA4 PORTC.3
#define DATA3 PORTC.2
#define DATA2 PORTC.1
#define DATA1 PORTC.0

#define mSTOP_DCM    DC1=0;DC2=0;
#define mRUN_DCM_CW  DC1=1;DC2=0;
#define mRUN_DCM_CCW DC1=0;DC2=1;

//typedef enum _dcmDir {DCM_CW, DCM_CCW}EDcmDir;
//typedef enum _stmDir {STM_CW, STM_CCW}EStmDir;
typedef enum _motDir {CW, CCW}EMotDir;
typedef enum _stmCmd {STOP_STM, RUN_STM_CW, RUN_STM_CCW}EStmCmd;
typedef enum _dcmSpeed {DCM_FAST, DCM_SLOW}EDcmSpeed;
typedef enum _dcmCmd {STOP_DCM, RUN_DCM_CW, RUN_DCM_CCW}EDcmCmd;

//typedef enum _dcmState {STOP_DCM, RUN_DCM_CW, RUN_DCM_CCW}EDcmState;

typedef enum states {INIT, CHK_SEN1, CHK_SEN2, SINGLE, COMPLEX, PRELIMINARY, SETTING, MAX_STATES}EState;
typedef enum events {PB1_PRESSED, SEN1_DETECTED, SEN2_DETECTED, PB2_PRESSED, ENSW_PRESSED, MAX_EVENTS}EEvent;
typedef EState (*evtHandler)(EState, EEvent);
typedef struct _dcmState {
  EDcmSpeed speed;
  EDcmCmd cmd;
  bool bUpdateCmd;
}DcmState;

DcmState dcmState ={DCM_FAST,STOP_DCM, false};

EStmCmd stmCmd = STOP_STM;
//EDcmCmd dcmCmd = STOP_DCM;
EState state = INIT;
EState lastState;

volatile uint16_t t2Counter;
uint8_t fnd1,fnd2,fnd3,fnd4;
bool led[22];
uint8_t totalAmount, currentAmount;
uint8_t fnd[]={0x3f,0x06,0x5b,0x4f, 0x66,0x6d,0x7c,0x07,
	       0x7f,0x67,0x79,0x0f, 0x39,0x3e,0x37,0x40};

#define numBtn 5
char btnPressed[numBtn];
int btnCounter[numBtn];
char bit_is_clear(char pinPort, char portBit)
{
  return !((pinPort>>portBit) & 0x01);
}
char isBtnPressed(int index,unsigned char pinPort,unsigned char Bitport,int numCount)
{
  //if (/*bit_is_clear(pinPort,portBit)*/1){
  if (bit_is_clear(pinPort,Bitport)){
    if (btnCounter[index]++> numCount){
      if (!btnPressed[index]){
	btnPressed[index]=1;
	return 1;
      }
      btnCounter[index]=0;
    }
  }
  else{
    btnPressed[index]= 0;
    btnCounter[index]=0;
  }
  return 0;
}
void BordInit()
{
  DDRA |= 0x01;  
  DDRB |= 0b00111111;
  DDRD |= 0b11111101;
  DDRC |= 0xFF;
  //PORTC= 0xff;
      
  UCSRB |= (1 << TXEN) ;
  UBRRL=51;
   
  TCCR0 |= (1<<CS02)  ;
  TCNT0=104;
  
  TCCR2 |= (1<<CS22)  ;
  TCNT2=0x83;

  TIMSK= (1<<TOIE2) | (1<<TOIE0);     
#asm("sei")
}
void drvFnd()
{
  static uint8_t idx=0;
  if (++idx > 4) {idx = 0;}
  switch (idx) {
  case 0:
    COM5=0 ;
    COM1=1 ;
    DATA7 = (fnd[fnd1] & (1 << 6)) >> 6;
    DATA6 = (fnd[fnd1] & (1 << 5)) >> 5;
    DATA5 = (fnd[fnd1] & (1 << 4)) >> 4;
    DATA4 = (fnd[fnd1] & (1 << 3)) >> 3;
    DATA3 = (fnd[fnd1] & (1 << 2)) >> 2;
    DATA2 = (fnd[fnd1] & (1 << 1)) >> 1;
    DATA1 = (fnd[fnd1] & (1 << 0)) >> 0;     
    break;
  case 1:
    COM1=0 ;
    COM2=1 ;
    DATA7 = (fnd[fnd2] & (1 << 6)) >> 6;
    DATA6 = (fnd[fnd2] & (1 << 5)) >> 5;
    DATA5 = (fnd[fnd2] & (1 << 4)) >> 4;
    DATA4 = (fnd[fnd2] & (1 << 3)) >> 3;
    DATA3 = (fnd[fnd2] & (1 << 2)) >> 2;
    DATA2 = (fnd[fnd2] & (1 << 1)) >> 1;
    DATA1 = (fnd[fnd2] & (1 << 0)) >> 0;
    break;
  case 2:
    COM2=0 ;
    COM3=1 ;
    DATA7 = (fnd[fnd3] & (1 << 6)) >> 6;
    DATA6 = (fnd[fnd3] & (1 << 5)) >> 5;
    DATA5 = (fnd[fnd3] & (1 << 4)) >> 4;
    DATA4 = (fnd[fnd3] & (1 << 3)) >> 3;
    DATA3 = (fnd[fnd3] & (1 << 2)) >> 2;
    DATA2 = (fnd[fnd3] & (1 << 1)) >> 1;
    DATA1 = (fnd[fnd3] & (1 << 0)) >> 0;
    break;
  case 3:
    COM3=0 ;
    COM4=1 ;
    DATA7 = (fnd[fnd4] & (1 << 6)) >> 6;
    DATA6 = (fnd[fnd4] & (1 << 5)) >> 5;
    DATA5 = (fnd[fnd4] & (1 << 4)) >> 4;
    DATA4 = (fnd[fnd4] & (1 << 3)) >> 3;
    DATA3 = (fnd[fnd4] & (1 << 2)) >> 2;
    DATA2 = (fnd[fnd4] & (1 << 1)) >> 1;
    DATA1 = (fnd[fnd4] & (1 << 0)) >> 0;
    break;
  case 4: 
    COM4=0;
    COM5=1;
    DATA6 = led[21] ? 1 : 0;
    DATA5 = led[20] ? 1 : 0;
    DATA4 = led[19] ? 1 : 0;
    DATA3 = led[18] ? 1 : 0;
    DATA2 = led[17] ? 1 : 0;
    DATA1 = led[16] ? 1 : 0;
    break;
  }
}
void drvLed()
{
  uint8_t i;
  static uint8_t theCounter = 0;
  if (++theCounter > 6) {
    theCounter = 0;
    str=0;
    for (i=0;i<16;i++) {
      data=led[i];clk=1;clk=0;
    }
    str=1;
  }
}
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
  TCNT0=104;
  drvLed();
  drvFnd();
}
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
  TCNT2=0x83;
  t2Counter++;
}
EState initPb1Pressed(EState s, EEvent e)
{
  uint16_t tCount = t2Counter;
  assert(s == INIT && e == PB1_PRESSED);
  printf("initPb1pressed\r\n");
  do
    {                                      
      if (t2Counter-tCount>3000) {
	dcmState.speed = DCM_FAST;
	dcmState.cmd = RUN_DCM_CCW;
	dcmState.bUpdateCmd = 1;
	fnd1 = 1;
	//mRUN_DCM_CCW;
	return CHK_SEN1;
      }
    } while (!PINA.1);
  return INIT;
}
EState chkSen1Sen1Detected(EState s, EEvent e)
{
  assert(s == CHK_SEN1 && e == SEN1_DETECTED);
  printf("chksen1sen1detected\r\n");
  mSTOP_DCM;
  delay_ms(1);
  mRUN_DCM_CW;
  delay_ms(27);
  mSTOP_DCM;
  delay_ms(1000);
  printf("Now the stpemottor is running CCW!!!\r\n");
  stmCmd = RUN_STM_CCW;
  return CHK_SEN2;
}
EState chkSen2Sen2Detected(EState s, EEvent e)
{
  assert(s == CHK_SEN2 && e == SEN2_DETECTED);
  printf("chksen2sen2detected\r\n");
  return SINGLE;
}
EState singlePb2Pressed(EState s, EEvent e)
{
  assert(s == SINGLE && e == PB2_PRESSED);
  printf("chksen2sen2detected\r\n");
  return COMPLEX;
}
EState complexPb2Pressed(EState s, EEvent e)
{
  assert(s == COMPLEX && e == PB2_PRESSED);
  printf("chksen2sen2detected\r\n");
  return PRELIMINARY;
}
EState prelimPb2Pressed (EState s, EEvent e)
{
  assert(s == PRELIMINARY && e == PB2_PRESSED);
  printf("chksen2sen2detected\r\n");
  return SINGLE;
}
EState singlEnswPressed (EState s, EEvent e)
{
  uint16_t tCount = t2Counter;
  assert(s == SINGLE && e == ENSW_PRESSED);
  printf("singlEnswPressed\r\n");
  do
    {                                      
      if (t2Counter-tCount>3000) {
	lastState = s;
	return SETTING;
      }
    } while (!ensw);
  return s;
}
EState complexEnswPressed (EState s, EEvent e)
{
  uint16_t tCount = t2Counter;
  assert(s == COMPLEX && e == ENSW_PRESSED);
  printf("complexEnswPressed\r\n");
  do
    {                                      
      if (t2Counter-tCount>3000) {
	lastState = s;
	return SETTING;
      }
    } while (!ensw);
  return s;
}
EState prelimEnswPressed (EState s, EEvent e)
{
  uint16_t tCount = t2Counter;
  assert(s == PRELIMINARY && e == ENSW_PRESSED);
  printf("prelimenswpressed\r\n");
  do
    {                                      
      if (t2Counter-tCount>3000) {
	lastState = s;
	return SETTING;
      }
    } while (!ensw);
  return s;
}
EState settingEnswPressed (EState s, EEvent e)
{
  uint16_t tCount = t2Counter;
  assert(s == SETTING && e == ENSW_PRESSED);
  printf("prelimenswpressed\r\n");
  do
    {                                      
      if (t2Counter-tCount>3000) {
	return lastState;
      }
    } while (!ensw);
  return s;
}
evtHandler transitions[MAX_STATES][ MAX_EVENTS] =
  {
   /*             PB11_PRESSED    SE1_DETECTED  SE2_DETECTED     PB2_PRESSED      ENSW_PRESSED*/
   /*[INIT ]*/    {initPb1Pressed  },
   /*[CHK_SEN1]*/ {(evtHandler)0, chkSen1Sen1Detected },
   /*[CHK_SEN2]*/ {(evtHandler)0, (evtHandler)0, chkSen2Sen2Detected },
   /*[SINGLE]*/   {(evtHandler)0, (evtHandler)0, (evtHandler)0, singlePb2Pressed , singlEnswPressed   },
   /*[COMPLEX]*/  {(evtHandler)0, (evtHandler)0, (evtHandler)0, complexPb2Pressed, complexEnswPressed },
   /*[PRELIMIN]*/ {(evtHandler)0, (evtHandler)0, (evtHandler)0, prelimPb2Pressed , prelimEnswPressed  },
   /*[SETTING]*/  {(evtHandler)0, (evtHandler)0, (evtHandler)0, prelimPb2Pressed , settingEnswPressed }
  };
void step_state(EEvent event)
{
  evtHandler handler = transitions[state][event];
  if (!handler){
    printf("Error fution Call is void[%d][%d]\n", state, event);
    return;}
  state = handler(state, event);
}
void RunStm(EMotDir dir)
{
  int8_t theCounter = 0;
  if (dir == CW) {
    if (++theCounter > 3) {
      theCounter = 0;
    }
  }else{
    if (--theCounter < 0) {
      theCounter = 3;
    }
  }

  switch (theCounter) {
  case 0: {sa=1;sb=0;sc=0;sd=0; break;}
  case 1: {sa=0;sb=1;sc=0;sd=0; break;}
  case 2: {sa=0;sb=0;sc=1;sd=0; break;}
  case 3: {sa=0;sb=0;sc=0;sd=1; break;}
  default: break;
  }
}
void main(void)
{
  uint16_t led19Counter, led19CounterLast = 0;
  uint16_t dcmCounter, dcmCounterLast = 0, dcmPwmCounter=0;
  uint16_t stmCounter, stmCounterLast = 0;
  uint16_t sen1Counter, sen1CounterLast = 0;
  uint16_t sen2Counter, sen2CounterLast = 0;
  bool sen1, sen2;
  bool mAState, mAlastState;//, mSen1, mSen2; 

  //  dcmState = {DCM_SLOW, STOP_DCM, false};
  BordInit();

  while (1) {
    // dc motor control
    if (dcmState.bUpdateCmd) {
      dcmState.bUpdateCmd = false; 
      if (dcmState.cmd == STOP_DCM) {
	mSTOP_DCM;
	led[20-1] = 0;
	led[22-1] = 0;
      }else if (dcmState.speed == DCM_FAST) {
	led[20-1] = 1;
	led[22-1] = 1;
	//#define RUN_DCM_CW  DC1=1;DC2=0;
	//#define RUN_DCM_CCW DC1=0;DC2=1;
	switch (dcmState.cmd) {
	case RUN_DCM_CW:  { DC1=1;DC2=0; break;}
	case RUN_DCM_CCW: { DC1=0;DC2=1; break;}
	default: break;
	}
      }else{
	led[20-1] = 0;
      }
    }
    if (dcmState.speed == DCM_SLOW) {
      dcmCounter = t2Counter;
      if (dcmCounter - dcmCounterLast>15) {
	dcmPwmCounter++;
	if (dcmPwmCounter%7 == 0) {
	  switch (dcmState.cmd) {
	  case RUN_DCM_CW:  {mRUN_DCM_CW; break;}
	  case RUN_DCM_CCW: {mRUN_DCM_CCW; break;}
	  default: break;
	  }
	}else{
	  mSTOP_DCM;
	}
	dcmCounterLast = dcmCounter;
      }
    }

    // stepper motor control
    stmCounter   = led19Counter = sen1Counter  = sen2Counter  = t2Counter;
    if (stmCounter - stmCounterLast>15) {
      switch (stmCmd) {
      case RUN_STM_CW:  { led[20-1] = 1;  RunStm(CW); break;}
      case RUN_STM_CCW: { led[20-1] = 1; RunStm(CCW); break;}
      case STOP_STM:    { led[20-1] = 0;  sa=sb=sc=sd=0;break;}
      default: break;
      }
      stmCounterLast = stmCounter;
    }
    //sen1
    if (sen1Counter - sen1CounterLast>1) {
      sen1 = SEN1;
      sen1CounterLast = sen1Counter;
    }
    if (!sen1 && SEN1) {
      if (state == CHK_SEN1) {
	printf("SEN1 Detected\r\n");
	step_state(SEN1_DETECTED);
      }
    }
    // sen2
    if (sen2Counter - sen2CounterLast>1) {
      sen2 = SEN2;
      sen2CounterLast = sen2Counter;
    }
    if (!sen2 && SEN2) {
      if (state == CHK_SEN2) {
	printf("SEN2 Detected\r\n");
	stmCmd = STOP_STM;
	step_state(SEN2_DETECTED);
      }
    }
    //pb1
    if (isBtnPressed(0, PINA, 1, 500)) {
      printf("state: %d\r\n", state);
      if (state == INIT) {
	step_state(PB1_PRESSED);
      }
    }
    // pb2
    if (isBtnPressed(1, PINA, 2, 500)) {
      printf("pb2 pressed\r\n");
      switch (state) {
      case SINGLE: {step_state(PB2_PRESSED); break;}
      case COMPLEX: {step_state(PB2_PRESSED); break;}
      case PRELIMINARY: {step_state(PB2_PRESSED); break;}
      default: break;
      }
    }
    //pb3
    if (isBtnPressed(2, PINA, 3, 500)) {
      printf("state: %d\r\n", state);
      if (state == INIT) {step_state(PB1_PRESSED);
      }
    }
    //pb4
    if (isBtnPressed(3, PINA, 4, 500)) {
      printf("state: %p\r\n", state);
      if (state == INIT) {
	step_state(PB1_PRESSED);
      }
    }
    // Encoder Switch
    if (isBtnPressed(4, PINA, 7, 500)) {
      printf("state: %p\r\n", state);
      if (state == INIT) {
	step_state(ENSW_PRESSED);
      }
    }
    //------------- rotary encoder spin event process
    mAState = ena;
    if (mAState != mAlastState ) {
      if (enb  != ena) {
	printf("CW, Rotaryencoder\r\n");
      }else{
	printf("CCW, Rotaryencoder\r\n");
      }
    }
    mAlastState = mAState;
    // led19
    if (led19Counter - led19CounterLast>500) {
      switch (state) {
      case SINGLE:      {led[19-1] = 0; break;}
      case COMPLEX:     {led[19-1] = 1; break;}
      case PRELIMINARY: {led[19-1] ^= 1; break;}
      default: break;
      }
      led19CounterLast = led19Counter;
    }
  }
}
