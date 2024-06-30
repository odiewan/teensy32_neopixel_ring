
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <led_pulse_train.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <serialPrint.h>
#include <neopixel_effects.h>
#include <npx_ring_event.h>

#define SS_SWITCH               24
#define SS_NEOPIX               6

#define SEESAW_ADDR             0x36

#define EEPROM_SIZE             512
#define EEPROM_DELAY            5

#define CLR_WHITE               0x00FFFFFF
#define CLR_OFF                 0x00000000
#define CLR_RED                 0x00FF0000
#define CLR_GREEN               0x0000FF00
#define CLR_BLUE                0x000000FF


#define BTN_PIN                 3
#define NPXL_PIN                2

#define NUM_NEOPIXELS           16

#define LED_ON_TIME             50
#define LED_OFF_TIME            750

#define DIR_DOWN                -1
#define DIR_NONE                0
#define DIR_UP                  1

#define LOOP_DELAY              5

#define SER_WAIT_TICKS          5
#define SER_WAIT_DELAY          250

#define SETUP_DELAY             1000
#define NUM_BYTES               32


#define NPX_ROT_DIR_STOP        0
#define NPX_ROT_DIR_CCW         1
#define NPX_ROT_DIR_CW          2

#define NPX_INC_MAX             64
#define NPX_AMP_MAX             255
#define NPX_AMP_MID             127
#define NPX_CALL_DELAY_CYCLES   15
#define NPX_CALL_INTV           8

#define DEF_NPX_INC             1
#define DEF_NPX_AMP_MIN         0
#define DEF_NPX_AMP_MAX         31

#define QUARTER_PI              float(PI / 4.0)
// #define RAD_TO_DEG float(180/PI)
// #define DEG_TO_RAD float(PI/80)

enum op_modes {
  OP_MD_BOOT,
  OP_MD_SETUP,
  OP_MD_ON,
  OP_MD_PATTERN_A,
  OP_MD_PATTERN_B,
  OP_MD_PATTERN_C,
  OP_MD_SET_NPX_MODE,
  OP_MD_SET_MAX,
  OP_MD_SET_MIN,
  OP_MD_SET_INT_R,
  OP_MD_SET_INT_G,
  OP_MD_SET_INT_B,
  NUM_OP_MODES,
  };

String op_mode_strs[] = {
  "BOOT",
  "SETUP",
  "ON",
  "OPT_A",
  "OPT_B",
  "OPT_C",
  "SET_NPX_M",
  "SET_MAX",
  "SET_MIN",
  "SET_INT_R",
  "SET_INT_G",
  "SET_INT_B",
  };


enum npx_modes {
  NPX_MD_OFF,
  NPX_MD_ON,
  NPX_MD_ASYC_SINE,
  NPX_MD_WHEEL,
  NPX_MD_WHEEL_SINGLE,
  NPX_MD_WHITE_SINE,
  NPX_MD_RED_SINE,
  NPX_MD_GREEN_SINE,
  NPX_MD_BLUE_SINE,

  NPX_MD_WHITE_STATIC,
  NPX_MD_RED_STATIC,
  NPX_MD_GREEN_STATIC,
  NPX_MD_BLUE_STATIC,
  NPX_MD_ORANGE_STATIC,

  NPX_MD_WHITE_STATIC_SNG,
  NPX_MD_RED_STATIC_SNG,
  NPX_MD_GREEN_STATIC_SNG,
  NPX_MD_BLUE_STATIC_SNG,
  NPX_MD_ORANGE_STATIC_SNG,
  NUM_NPX_MODES,
  };

String npx_mode_strs[] = {
  "OFF",
  "ON",
  "ASYC sine",
  "WHL All",
  "WHL Single",
  "Wht sine",
  "Rd sine",
  "Grn sine",
  "Blu sine",
  "Wht ",
  "Rd",
  "Grn",
  "Blu",
  "Org",
  "Wht StSng",
  "Rd StSng",
  "Grn StSng",
  "Blu StSng",
  "Org StSng",

  };

enum eeprom_registers {
  EE_REG_NEOPIXEL_MODE,
  EE_REG_AMP_MAX,
  EE_REG_AMP_MIN,
  EE_REG_R_INT,
  EE_REG_G_INT,
  EE_REG_B_INT,
  NUM_EEPROM_REG,
  };

uint8_t eeprom[EEPROM_SIZE] = {};

uint8_t eeprom_live[EEPROM_SIZE] = {};
char inByteBuffer[NUM_BYTES] = {};



int x;
int dir;
int npxlMode;
int npxlModeReq;
int opMd;
bool setting_mode;
int btn_dwn_tmr;
int32_t npxIdx;
int32_t enc_ovr_tmr;
int32_t enc_ovr_cmd;


int def_count = 0;
int dir_up_count = 0;
int dir_dn_count = 0;
int amp_sel = 0;


colorVector cAll = { 0,0,0 };

bool npxEnAry[NUM_NEOPIXELS] = {
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  };



String inStr = "";

int64_t iCount = 0;

Adafruit_seesaw ss;
Adafruit_NeoPixel strip(NUM_NEOPIXELS, NPXL_PIN, NEO_GRB + NEO_KHZ800);
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t enc_position;
int32_t new_position;
int32_t enc_delta;

bool nxplEn;

bool btnDIN;
bool btnDINShadow;
bool btnShortPress;
bool btnShortShadow;
bool btnLongPress;
bool btnOState;
bool btnStateR;
bool btnStateG;
bool btnStateB;
bool btnStateRG;
bool enc_ovr;


neopixel_color npxR;
neopixel_color npxG;
neopixel_color npxB;

npx_ring_event nre0;
npx_ring_event nreR;
npx_ring_event nreG;
npx_ring_event nreB;
npx_ring_event nreRG;

uint32_t npxColor;
uint8_t _wheelPos;

int npx_op_mode_mtrx[NUM_OP_MODES] = {
  NPX_MD_OFF,               //  OP_MD_BOOT
  NPX_MD_OFF,               //  OP_MD_SETUP
  NPX_MD_ON,                //  OP_MD_ON
  NPX_MD_ORANGE_STATIC,     //  OP_MD_PATTERN_A
  NPX_MD_WHEEL,             //  OP_MD_PATTERN_B
  NPX_MD_WHEEL_SINGLE,      //  OP_MD_PATTERN_C
  NPX_MD_WHEEL_SINGLE,      //  OP_MD_SET_NPX_MO
  NPX_MD_WHITE_STATIC_SNG,  //  OP_MD_SET_MAX
  NPX_MD_ORANGE_STATIC_SNG, //  OP_MD_SET_MIN
  NPX_MD_RED_STATIC_SNG,    //  OP_MD_SET_INT_R
  NPX_MD_GREEN_STATIC_SNG,  //  OP_MD_SET_INT_G
  NPX_MD_BLUE_STATIC_SNG,   //  OP_MD_SET_INT_B
  };


//=================================================================================================
void ledToggle() {
  static bool bit = false;
  bit = !bit;
  digitalWrite(LED_BUILTIN, bit);
  }

//-----------------------------------------------------------------------------
String recvWithEndMarker() {
  static int bCnt = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();
    if (bCnt < NUM_BYTES - 1) {
      if (rc != endMarker) {
        inByteBuffer[bCnt] = rc;
      }
      bCnt++;
    }
    else{
      serPrntNL("buffer overflow");
    }
  }

  if (bCnt > 0) {
    serPrntVNL("Rx'ed ", bCnt, " bytes");
    inByteBuffer[bCnt] = '\0';
    inStr = inByteBuffer;
  }
  else
    inStr = "";

  bCnt = 0;

  return inStr;
}

//=================================================================================================
void readEEPROM() {
  serPrntNL("readEEPROM()");
  String tmpStr;
  int byte_read_cnt = 0;

  for (int i = 0; i < NUM_EEPROM_REG; i++) {
    eeprom[i] = EEPROM.read(i);
    eeprom_live[i] = eeprom[i];
    tmpStr = "readEEPROM()[";
    tmpStr += i;
    tmpStr += "]:";
    tmpStr += eeprom_live[i];
    byte_read_cnt++;
    serPrntNL(tmpStr);
    }
  // memcpy(eeprom_live, eeprom, sizeof(eeprom_live));

  // calcColorRange();

  serPrntNL();
  tmpStr = "Read ";
  tmpStr += byte_read_cnt;
  tmpStr += " of ";
  tmpStr += EEPROM_SIZE;
  tmpStr += " bytes from EEPROM";
  serPrntNL(tmpStr);
  }


//=================================================================================================
void serPrintLiveEEPROM() {
  String tmpStr;
  for (int i = 0; i < NUM_EEPROM_REG; i++) {
    tmpStr = "serPrintLiveEEPROM()[";
    tmpStr += i;
    tmpStr += "]:";
    tmpStr += eeprom_live[i];
    tmpStr += " e:";
    tmpStr += eeprom[i];
    serPrntNL(tmpStr);
    }
  }

//=================================================================================================
void writeEEPROM() {
  serPrntNL("writeEEPROM()");
  String tmpStr;
  int byte_write_cnt = 0;

  for (int i = 0; i < NUM_EEPROM_REG; i++) {

    byte_write_cnt += writeEepromReg(i);
    delay(EEPROM_DELAY);
  }

  if (byte_write_cnt > 0) {
    tmpStr = "Wrote ";
    tmpStr += byte_write_cnt;
    tmpStr += " of ";
    tmpStr += EEPROM_SIZE;
    tmpStr += " bytes from EEPROM";
  }
  else {
    tmpStr = "No changes to EEPROM";
  }

  serPrntNL(tmpStr);
}

//=================================================================================================
int writeEepromReg(uint16_t nIdx) {
  serPrnt("writeEepromReg():");
  String tmpStr;
  if (eeprom_live[nIdx] != eeprom[nIdx]) {
    eeprom[nIdx] = eeprom_live[nIdx];
    EEPROM.write(nIdx, eeprom[nIdx]);
    tmpStr = "Register changed: writeEepromReg[";
    tmpStr += nIdx;
    tmpStr += "]:";
    tmpStr += eeprom[nIdx];
    serPrntNL(tmpStr);
    return 1;
    }
  else {
    tmpStr = "Idx:";
    tmpStr += nIdx;
    tmpStr += ": No change to register";
    serPrntNL(tmpStr);
    return 0;
    }
  }

//=================================================================================================
void setup() {
  int ser_wait_cnt = 0;
  opMd = OP_MD_BOOT;
  npxlModeReq = NPX_MD_ASYC_SINE;
  enc_ovr = false;
  enc_ovr_cmd = 0;
  btnDIN = false;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);

  Serial.begin(9600);
  ledPulseTrain(3);
  while (!Serial && ser_wait_cnt < SER_WAIT_TICKS) {
    ser_wait_cnt++;
    ledToggle();
    delay(SER_WAIT_DELAY);
  }
  serPrntNL("Serial OK");



  ledPulseTrain(4);
  serPrntNL("Read EEPROM contents");
  readEEPROM();

  npxlMode = eeprom_live[EE_REG_NEOPIXEL_MODE];

  strip.begin();

  for(int i = 0; i < strip.numPixels(); i++){
    strip.fill(strip.Color(eeprom_live[EE_REG_AMP_MAX], 0, 0), 0, 5);
    strip.fill(strip.Color(0, eeprom_live[EE_REG_AMP_MAX], 0), 5, 10);
    strip.fill(strip.Color(0, 0, eeprom_live[EE_REG_AMP_MAX]), 10, strip.numPixels());
  }

  strip.show();


  ledPulseTrain(2);
  npxR = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
  npxG = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
  npxB = neopixel_color(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);


  ledPulseTrain(4);
  nre0 = npx_ring_event(&btnOState, (uint32_t)0);
  nreR = npx_ring_event(&btnStateR, (uint32_t)0x00FF0000);
  nreG = npx_ring_event(&btnStateG, (uint32_t)0x0000FF00);
  nreB = npx_ring_event(&btnStateB, (uint32_t)0x000000FF);
  nreRG = npx_ring_event(&btnStateRG, (uint32_t)0x00FFFF00);


  ledPulseTrain(6);


  Serial.println("Looking for seesaw!");

  if (!ss.begin(SEESAW_ADDR) || !sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
    }
  Serial.println("seesaw started");


  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  enc_position = ss.getEncoderPosition();
  new_position = enc_position;

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();

  sspixel.setBrightness(20);
  sspixel.show();

  delay(SETUP_DELAY);


  opMd = OP_MD_ON;
  serPrntNL("Setup done");

  }



//-----------------------------------------------------------------------------
void paramSetColorHandler(String nCmd, String nParamName, int& nParam, int nVal) {
  paramSetHandler(nCmd, nParamName, nParam, nVal, NPX_AMP_MAX, 0);
  }

//-----------------------------------------------------------------------------
void paramSetHandler(String nCmd, String nParamName, int& nParam, int nVal, int nUpLim, int nLoLim) {
  if (inStr == nCmd) {
    serPrntNL(nCmd + ": set " + nParamName);
    nParam = nVal;

    if (nParam > nUpLim)
      nParam = nLoLim;
    else if (nParam < nLoLim)
      nParam = nUpLim;
    }
  }

//-----------------------------------------------------------------------------
void paramIncColorHandler(String nCmd, String nParamName, int& nParam, int nInc) {
  paramIncHandler(nCmd, nParamName, nParam, nInc, NPX_AMP_MAX, 0);
  }

//-----------------------------------------------------------------------------
void paramIncColorHandler(String nCmd, String nParamName, float& nParam, int nInc) {
  paramIncHandler(nCmd, nParamName, nParam, nInc, NPX_AMP_MAX, 0);
  }

//-----------------------------------------------------------------------------
void paramIncHandler(String nCmd, String nParamName, int& nParam, int nInc, int nUpLim, int nLoLim) {
  if (inStr == nCmd) {
    serPrntNL("i:" + nCmd + ": increment " + nParamName);

    nParam += nInc;
    serPrntNL("i:" + nCmd + ": nParam " + nParam);

    if (nParam > nUpLim)
      nParam = nLoLim;
    else if (nParam < nLoLim)
      nParam = nUpLim;
  }
}

//-----------------------------------------------------------------------------
void paramIncHandler(int nIdx, int nInc, int nUpLim, int nLoLim) {
  int _param = eeprom_live[nIdx];

  _param += nInc;

  if (_param > nUpLim)
    _param = nLoLim;
  else if (_param < nLoLim)
    _param = nUpLim;

  eeprom_live[nIdx] = _param;
}

//-----------------------------------------------------------------------------
void paramIncHandler(String nCmd, String nParamName, float& nParam, float nInc, float nUpLim, float nLoLim) {
  if (inStr == nCmd) {
    serPrntNL("f:" + nCmd + ": increment " + nParamName);

    nParam += nInc;
    serPrntNL("f:" + nCmd + ": nParam " + nParam);

    if (nParam > nUpLim)
      nParam = nLoLim;
    else if (nParam < nLoLim)
      nParam = nUpLim;
    }
  }


//=================================================================================================
void setAllNeoPixels() {
  for (int i = 0; i < strip.numPixels(); i++) {
    if (npxEnAry[i])
      strip.setPixelColor(i, strip.Color(cAll.r, cAll.g, cAll.b));
    }

  strip.show();
  }

//=================================================================================================
void setAllNeoPixels(uint32_t nColor) {
  // strip.fill(strip.Color(cAll.r, cAll.g, cAll.b), 0, strip.numPixels());

  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (npxEnAry[i])
      strip.setPixelColor(i, nColor);
    else
      strip.setPixelColor(i, 0);
    }

  strip.show();
  }


//=================================================================================================
uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return sspixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return sspixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
  WheelPos -= 170;
  return sspixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }


// //=================================================================================================
// void setAllNeoPixels() {
//   strip.fill(npxColor, 0, strip.numPixels());
//   strip.show();
// }

//-----------------------------------------------------------------------------
void handleSerIn() {
  if (recvWithEndMarker() > "") {

    if (inStr == "non") {
      nxplEn = true;
      serPrntNL("on: neopixel ring on");
      }

    else if (inStr == "noff") {
      nxplEn = false;
      serPrntNL("off: neopixel ring off");
    }

    else if (inStr == "save") {
      serPrntNL("save: save eeprom");
      writeEEPROM();
      btnStateG = true;
      serPrntNL("save: eeprom saved");
    }

    else if (inStr == "init") {
      serPrntNL("init: init eeprom");
      eeprom_live[EE_REG_NEOPIXEL_MODE] = 1;
      eeprom_live[EE_REG_AMP_MAX] = 127;
      eeprom_live[EE_REG_AMP_MIN] = 100;
      eeprom_live[EE_REG_R_INT] = 1;
      eeprom_live[EE_REG_G_INT] = 2;
      eeprom_live[EE_REG_B_INT] = 3;
      serPrntNL("writing to eeprom");
      delay(250);
      writeEEPROM();
      btnStateG = true;
      delay(250);
      serPrntNL("save: eeprom saved");
      serPrintLiveEEPROM();

    }
    else if (inStr == "read") {
      serPrntNL("read: read eeprom");
      readEEPROM();
      // calcColorRange();
      btnOState = true;
      serPrntNL("read: eeprom read");

      }
    else if (inStr == "prnt") {
      serPrntNL("prnt: read eeprom");
      serPrintLiveEEPROM();

      }

  //   else if (inStr == "btn") {
  //     serPrntNL("btn: sim button");
  //     btnOState = true;
  //     }
  //   else if (inStr == "btnR") {
  //     serPrntNL("btnR: sim button");
  //     btnStateR = true;
  //     }
  //   else if (inStr == "btnG") {
  //     serPrntNL("btnG: sim button");
  //     btnStateG = true;
  //     }
  //   else if (inStr == "btnB") {
  //     serPrntNL("btnB: sim button");
  //     btnStateB = true;
  //     }
  //   else if (inStr == "btnRG") {
  //     serPrntNL("btnRG: sim button");
  //     btnStateRG = true;
  //     }

    inStr = "";
    }
  }

//=================================================================================================
void taskSerOut() {
  String _tmpStr = "";

  _tmpStr = "iC:";
  _tmpStr += iCount;

  _tmpStr += " opMd:";
  _tmpStr += opMd;
  _tmpStr += " (" + op_mode_strs[opMd] + ")";

  _tmpStr += " npxlMd:";
  _tmpStr += npxlMode;
  _tmpStr += " (" + npx_mode_strs[npxlMode] + ")";

  // _tmpStr += " btnDIN:";
  // _tmpStr += btnDIN;

  _tmpStr += " btnS:";
  _tmpStr += btnShortPress;

  _tmpStr += " btnL:";
  _tmpStr += btnLongPress;

  _tmpStr += " enc:";
  _tmpStr += enc_position;

  // _tmpStr += " dEnc:";
  // _tmpStr += enc_delta;

  // _tmpStr += " enc_ovr_cmd:";
  // _tmpStr += enc_ovr_cmd;


  // _tmpStr += " enc_ovr:";
  // _tmpStr += enc_ovr;

  // _tmpStr += " _wPos:";
  // _tmpStr += _wheelPos;


  _tmpStr += " | npx:";
  _tmpStr += npxIdx;



  _tmpStr += " EE :max:";
  _tmpStr += eeprom_live[EE_REG_AMP_MAX];
  _tmpStr += " min:";
  _tmpStr += eeprom_live[EE_REG_AMP_MIN];


  _tmpStr += " incR:";
  _tmpStr += eeprom_live[EE_REG_R_INT];
  _tmpStr += "  incG:";
  _tmpStr += eeprom_live[EE_REG_G_INT];
  _tmpStr += " incB:";
  _tmpStr += eeprom_live[EE_REG_B_INT];

  // _tmpStr += " btnO";
  // _tmpStr += btnOState;

  // _tmpStr += " btnR";
  // _tmpStr += btnStateR;

  _tmpStr += " dwn_tmr:";
  _tmpStr += btn_dwn_tmr;

  _tmpStr += " | set_md:";
  _tmpStr += setting_mode;

  // _tmpStr += " nreR.Color:";
  // _tmpStr += nreR.nreColor;
  // _tmpStr += " R:";
  // _tmpStr += nreR.nreR;
  // _tmpStr += " G:";
  // _tmpStr += nreR.nreG;
  // _tmpStr += " B:";
  // _tmpStr += nreR.nreB;

  // _tmpStr += " nreG.Color:";
  // _tmpStr += nreG.nreColor;
  // _tmpStr += " R:";
  // _tmpStr += nreG.nreR;
  // _tmpStr += " G:";
  // _tmpStr += nreG.nreG;
  // _tmpStr += " B:";
  // _tmpStr += nreG.nreB;

  // _tmpStr += " nreB.Color:";
  // _tmpStr += nreB.nreColor;
  // _tmpStr += " B:";
  // _tmpStr += nreB.nreB;
  // _tmpStr += " G:";
  // _tmpStr += nreB.nreG;
  // _tmpStr += " B:";
  // _tmpStr += nreB.nreB;

  // _tmpStr += " npxClr:";
  // _tmpStr += npxColor;

  // _tmpStr += " r:";
  // _tmpStr += (npxColor & 0x00FF0000) >> 16;
  // _tmpStr += " g:";
  // _tmpStr += (npxColor & 0x0000FF00) >> 8;
  // _tmpStr += " b:";
  // _tmpStr += (npxColor & 0x000000FF);

  serPrntNL(_tmpStr);

  }

//=================================================================================================
void taskNpxModeHandler() {
  // for(int i = 0; i < NUM_OP_MODES; i++)
    npxlMode = npx_op_mode_mtrx[opMd];

  // switch (opMd) {
  //   default:
  //   case OP_MD_BOOT:
  //   case OP_MD_SETUP:
  //     break;

  //   case OP_MD_ON:
  //     npxlMode = NPX_MD_ON;
  //     break;

  //   case OP_MD_PATTERN_A:
  //     npxlMode = eeprom_live[EE_REG_NEOPIXEL_MODE];
  //     break;

  //   case OP_MD_PATTERN_B:
  //     npxlMode = NPX_MD_WHEEL;
  //     break;

  //   case OP_MD_PATTERN_C:
  //     npxlMode = NPX_MD_WHEEL_SINGLE;
  //     break;

  //   case OP_MD_SET_NPX_MODE:
  //     npxlMode = NPX_MD_WHEEL_SINGLE;
  //     break;

  //   case OP_MD_SET_MAX:
  //     npxlMode = NPX_MD_WHITE_STATIC;
  //     break;

  //   case OP_MD_SET_MIN:
  //     npxlMode = NPX_MD_GREEN_STATIC;

  //     break;
  // }
}

//=================================================================================================
void enNpxl(int32_t nIdx) {
  static int32_t _tmpIdx = 0;
  _tmpIdx = strip.numPixels() - npxIdx;
  _tmpIdx = (_tmpIdx & 0x0F);
  npxEnAry[_tmpIdx] = true;
}


//=================================================================================================
void taskNeopixelRing() {
  static int _tmpIdx = 0;

  switch (npxlMode) {
    default:
    case NPX_MD_OFF:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = 1;
      cAll.g = 1;
      cAll.b = 1;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    case NPX_MD_ON:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = eeprom_live[EE_REG_AMP_MAX];
      cAll.b = eeprom_live[EE_REG_AMP_MAX];
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    case NPX_MD_ASYC_SINE:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = npxR.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      cAll.g = npxG.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      cAll.b = npxB.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);

      break;

    case NPX_MD_WHEEL:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      _wheelPos++;
      npxColor = Wheel(_wheelPos);
      break;


    case NPX_MD_WHEEL_SINGLE:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      _wheelPos++;
      npxColor = Wheel(_wheelPos);

      npxIdx--;

      enNpxl(npxIdx);
      break;

    case NPX_MD_RED_SINE:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = npxR.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      cAll.g = 0;
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    case NPX_MD_GREEN_SINE:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = 0;
      cAll.g = npxG.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    case NPX_MD_BLUE_SINE:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = 0;
      cAll.g = 0;
      cAll.b = npxB.npcLedSine(eeprom_live[EE_REG_AMP_MIN], eeprom_live[EE_REG_AMP_MAX]);
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    //=====================================================
    case NPX_MD_WHITE_STATIC:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = eeprom_live[EE_REG_AMP_MAX];
      cAll.b = eeprom_live[EE_REG_AMP_MAX];
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);

      break;

    case NPX_MD_RED_STATIC:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = 0;
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);

      break;

    case NPX_MD_GREEN_STATIC:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = 0;
      cAll.g = eeprom_live[EE_REG_AMP_MIN];
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;

    case NPX_MD_BLUE_STATIC:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = 0;
      cAll.g = 0;
      cAll.b = eeprom_live[EE_REG_AMP_MAX];
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      break;


    case NPX_MD_ORANGE_STATIC:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), true);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = eeprom_live[EE_REG_AMP_MAX];
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);

      break;


    case NPX_MD_WHITE_STATIC_SNG:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = eeprom_live[EE_REG_AMP_MAX];
      cAll.b = eeprom_live[EE_REG_AMP_MAX];
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);

      enNpxl(npxIdx);
      break;

    case NPX_MD_RED_STATIC_SNG:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      cAll.r = eeprom_live[EE_REG_AMP_MAX];
      cAll.g = 0;
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      enNpxl(npxIdx);
      break;

    case NPX_MD_GREEN_STATIC_SNG:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      cAll.r = 0;
      cAll.g = eeprom_live[EE_REG_AMP_MIN];
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      enNpxl(npxIdx);
      break;

    case NPX_MD_BLUE_STATIC_SNG:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      cAll.r = 0;
      cAll.g = 0;
      cAll.b = eeprom_live[EE_REG_AMP_MAX];
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      enNpxl(npxIdx);
      break;


    case NPX_MD_ORANGE_STATIC_SNG:
      std::fill(std::begin(npxEnAry), std::end(npxEnAry), false);
      cAll.r = eeprom_live[EE_REG_AMP_MIN];
      cAll.g = eeprom_live[EE_REG_AMP_MIN];
      cAll.b = 0;
      npxColor = strip.Color(cAll.r, cAll.g, cAll.b);
      enNpxl(npxIdx);
      break;
  }

  if (amp_sel > 255)
    amp_sel = 0;

  if (amp_sel < 0)
    amp_sel = 255;



  strip.clear();
  // rTmpFloat = 10;
  // rTmpFloat *= i/16.0;
  // rTmpFloat *= 2 * PI;
  // rTmpFloat = sin(rTmpFloat);



  // npxColor = nre0.nreBtnOvr(&cAll);
  // npxColor = nreR.nreBtnOvr(&cAll);
  // npxColor = nreG.nreBtnOvr(&cAll);
  // npxColor = nreB.nreBtnOvr(&cAll);
  // npxColor = nreRG.nreBtnOvr(&cAll);

  setAllNeoPixels(npxColor);
  }

//=================================================================================================
void taskModeHandler() {
  static bool setting_mode_shadow;

  if (setting_mode) {
    if (setting_mode_shadow != setting_mode)
      opMd = OP_MD_SET_NPX_MODE;
    else {
      if (opMd >= NUM_OP_MODES)
        opMd = OP_MD_SET_NPX_MODE;
    }
  }
  else {
    if (setting_mode_shadow != setting_mode)
      opMd = OP_MD_ON;
    else {
      if (opMd > OP_MD_PATTERN_C)
        opMd = OP_MD_ON;
    }
  }

  setting_mode_shadow = setting_mode;
}


//=================================================================================================
int32_t knob_to_npx_id(){
  static int32_t _idx = 0;
  if (enc_delta != 0) {
    enc_ovr = true;
    enc_ovr_tmr = 100;

    }

  if (enc_ovr) {
    enc_ovr_cmd = (int32_t)strip.numPixels() - (enc_position % (int32_t)strip.numPixels());
    _idx = (enc_ovr_cmd & 0x0F);

    enc_ovr_tmr--;
    if (enc_ovr_tmr == 0)
      enc_ovr = false;
    }
  return _idx;
}

//=================================================================================================
void handle_button_press() {
  btnDIN = !ss.digitalRead(SS_SWITCH);
  if (btnDIN) {
    btn_dwn_tmr++;
  }
  else if (btnDINShadow != btnDIN) {
    if(btn_dwn_tmr > 100) {
      btnLongPress = true;
      btnShortPress = false;
    }
    else{
      btnLongPress = false;
      btnShortPress = true;
    }
    btn_dwn_tmr = 0;
  }
  else {
    btnLongPress = false;
    btnShortPress = false;
    btn_dwn_tmr = 0;
  }

  //---handle button press in each op mode-------------------------------------
  switch (opMd) {
    default:
      break;

    case OP_MD_ON:
    case OP_MD_PATTERN_A:
    case OP_MD_PATTERN_B:
    case OP_MD_PATTERN_C:
      if(btnLongPress){
        setting_mode = true;
        btnStateR = true;

      }
      if (btnShortPress)
        opMd++;
      break;

    case OP_MD_SET_NPX_MODE:
    case OP_MD_SET_MAX:
    case OP_MD_SET_MIN:
    case OP_MD_SET_INT_R:
    case OP_MD_SET_INT_G:
    case OP_MD_SET_INT_B:
      if (btnLongPress) {
        setting_mode = false;
        btnStateRG = true;
        writeEEPROM();
        btnStateRG = true;
        }
      if (btnShortPress)
        opMd++;
      break;
    }

  btnDINShadow = btnDIN;
}

//=================================================================================================
void handleEncoder() {
  new_position = ss.getEncoderPosition();

  //--- calculate encoder data
  if (enc_position != new_position) {

    enc_delta = new_position - enc_position;
    enc_position = new_position;
  }
  else
    enc_delta = 0;

  switch (opMd) {
    default:
      break;

    case OP_MD_ON:
    case OP_MD_PATTERN_A:
    case OP_MD_PATTERN_B:
    case OP_MD_PATTERN_C:
      npxIdx = knob_to_npx_id();
      break;

    case OP_MD_SET_NPX_MODE:
      paramIncHandler(EE_REG_NEOPIXEL_MODE, enc_delta, NUM_NPX_MODES, 0);
      npxIdx = map(eeprom_live[EE_REG_NEOPIXEL_MODE], 0, NUM_NPX_MODES, 0, strip.numPixels());
      break;

    case OP_MD_SET_MAX:
      paramIncHandler(EE_REG_AMP_MAX, enc_delta, 255, 1);
      npxIdx = map(eeprom_live[EE_REG_AMP_MAX], 0, 255, 0, strip.numPixels());
      break;

    case OP_MD_SET_MIN:
      paramIncHandler(EE_REG_AMP_MIN, enc_delta, 64, 1);
      npxIdx = map(eeprom_live[EE_REG_AMP_MIN], 0, 64, 0, strip.numPixels());
      break;

    case OP_MD_SET_INT_R:
      paramIncHandler(EE_REG_R_INT, enc_delta, 64, 1);
      npxIdx = map(eeprom_live[EE_REG_R_INT], 1, 64, 0, strip.numPixels());
      break;

    case OP_MD_SET_INT_G:
      paramIncHandler(EE_REG_G_INT, enc_delta, 64, 1);
      npxIdx = map(eeprom_live[EE_REG_G_INT], 1, 64, 0, strip.numPixels());
      break;

    case OP_MD_SET_INT_B:
      paramIncHandler(EE_REG_B_INT, enc_delta, 64, 1);
      npxIdx = map(eeprom_live[EE_REG_B_INT], 1, 64, 0, strip.numPixels());
      break;

  }
}

//=================================================================================================
void loop() {
  handle_button_press();
  handleEncoder();
  handleSerIn();

  taskModeHandler();

  if (iCount % NPX_CALL_INTV == 0 && iCount > NPX_CALL_DELAY_CYCLES) {
    taskNpxModeHandler();
    taskNeopixelRing();

  }

  if (iCount % SER_WAIT_TICKS == 0)
    taskSerOut();

  iCount++;
  delay(LOOP_DELAY);

}