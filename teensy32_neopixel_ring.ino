
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <led_pulse_train.h>
#include <Adafruit_NeoPixel.h>
#include <serialPrint.h>
#include <neopixel_effects.h>
#include <npx_ring_event.h>

#define EEPROM_SIZE     512




#define NPXL_PIN        2
#define NUM_NEOPIXELS        16

#define LED_ON_TIME     50
#define LED_OFF_TIME    750

#define DIR_DOWN        -1
#define DIR_NONE        0
#define DIR_UP          1


#define SER_WAIT_TICKS  10

#define SETUP_DELAY     1000
#define NUM_BYTES       32


#define NPX_ROT_DIR_STOP        0
#define NPX_ROT_DIR_CCW         1
#define NPX_ROT_DIR_CW          2

#define DEF_NPX_INC     1
#define DEF_NPX_AMP_MIN 0
#define DEF_NPX_AMP_MAX 31

#define QUARTER_PI float(PI / 4.0)
// #define RAD_TO_DEG float(180/PI)
// #define DEG_TO_RAD float(PI/80)


enum npx_modes {
  NPX_MD_OFF,
  NPX_MD_ASYC_SINE,
  NPX_MD_RED_SINE,
  NPX_MD_GREEN_SINE,
  NPX_MD_BLUE_SINE,
  NPX_MD_STATIC,
  NPX_MD_STATIC_RED,
  NPX_MD_STATIC_GREEN,
  NPX_MD_STATIC_BLUE,
  NUM_NPX_MODES,
  };

String npx_mode_strs[] = {
  "OFF",
  "ASYC_SINE",
  "RED_SINE",
  "GREEN_SINE",
  "BLUE_SINE",
  "STATIC",
  };

enum eeprom_registers {
  EE_REG_NEOPIXEL_MODE,
  EE_REG_R_MAX,
  EE_REG_G_MAX,
  EE_REG_B_MAX,
  EE_REG_R_MIN,
  EE_REG_G_MIN,
  EE_REG_B_MIN,
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
int npxlMode = NPX_MD_ASYC_SINE;
uint16_t idx = 0;
uint8_t npxl_rotation_dir = true;

int def_count = 0;
int dir_up_count = 0;
int dir_dn_count = 0;
int azim = 0;


colorVector c00 = { 0,0,0 };
float incR = 2;
float incG = 4;
float incB = 8;
uint8_t r = 0;
uint8_t b = 0;
uint8_t g = 0;
float rRads = 0;

String inStr = "";

int64_t iCount = 0;

Adafruit_NeoPixel strip(NUM_NEOPIXELS, NPXL_PIN, NEO_GRB + NEO_KHZ800);
bool nxplEn = true;

bool btnState = false;
bool btnStateR = false;
bool btnStateG = false;
bool btnStateB = false;
bool btnStateRG = false;

neopixel_color npxR;
neopixel_color npxG;
neopixel_color npxB;

npx_ring_event nre0;
npx_ring_event nreR;
npx_ring_event nreG;
npx_ring_event nreB;
npx_ring_event nreRG;


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
    else
      serPrntNL("buffer overflow");
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
    delay(5);
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
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  ledPulseTrain(3);
  while (!Serial && ser_wait_cnt < 10) {
    ser_wait_cnt++;
    ledToggle();
    delay(250);
    }
  serPrntNL("Serial OK");

  ledPulseTrain(4);
  serPrntNL("Read EEPROM contents");
  readEEPROM();

  npxlMode = eeprom_live[EE_REG_NEOPIXEL_MODE];

  strip.begin();
  strip.setPixelColor(0, eeprom_live[EE_REG_R_MAX], 0, 0);
  strip.setPixelColor(8, 0, eeprom_live[EE_REG_R_MAX], 0);
  strip.setPixelColor(15, 0, 0, eeprom_live[EE_REG_R_MAX]);
  strip.show();


  ledPulseTrain(2);
  npxR = neopixel_color(eeprom_live[EE_REG_R_INT], eeprom_live[EE_REG_R_MAX]);
  npxG = neopixel_color(eeprom_live[EE_REG_R_INT], eeprom_live[EE_REG_G_MAX]);
  npxB = neopixel_color(eeprom_live[EE_REG_R_INT], eeprom_live[EE_REG_B_MAX]);


  ledPulseTrain(4);

  nre0 = npx_ring_event(&btnState, 0, 0, 0);
  nreR = npx_ring_event(&btnStateR, 255, 0, 0);
  nreG = npx_ring_event(&btnStateG, 0, 255, 0);
  nreB = npx_ring_event(&btnStateB, 0, 0, 255);
  nreRG = npx_ring_event(&btnStateRG, 255, 255, 0);


  ledPulseTrain(6);


  delay(SETUP_DELAY);

  serPrntNL("Setup done");
  }

//=================================================================================================
void taskSerOut() {
  String _tmpStr = "";
  uint8_t tmpInt = 0;


  if (iCount % 3 == 0) {

    tmpInt = (c00.r + c00.g + c00.b);
    _tmpStr = "--iC:";
    _tmpStr += iCount;
    // _tmpStr += " azim:";
    // _tmpStr += azim;
    // _tmpStr += " x:";
    // _tmpStr += x;
    _tmpStr += " npxlMode:";
    _tmpStr += npxlMode;


    _tmpStr += " rR:";
    _tmpStr += eeprom_live[EE_REG_R_MAX];
    _tmpStr += " rG:";
    _tmpStr += eeprom_live[EE_REG_G_MAX];
    _tmpStr += " rB:";
    _tmpStr += eeprom_live[EE_REG_B_MAX];


    _tmpStr += " incR:";
    _tmpStr += incR;
    _tmpStr += "  incG:";
    _tmpStr += incG;
    _tmpStr += " incB:";
    _tmpStr += incB;

    // _tmpStr += " nreR.nreEnShadow";
    // _tmpStr += nreR.nreEnShadow;

    // _tmpStr += " nreR.nreEn";
    // _tmpStr += nreR.nreEn;

    // _tmpStr += " nreR.nreTmr";
    // _tmpStr += nreR.nreTmr;

    // _tmpStr += " btnState";
    // _tmpStr += btnState;

    // _tmpStr += " btnRState";
    // _tmpStr += btnStateR;

    // _tmpStr += " btnGState";
    // _tmpStr += btnStateG;

    // _tmpStr += " btnBState";
    // _tmpStr += btnStateB;




    _tmpStr += " r:";
    _tmpStr += c00.r;
    _tmpStr += " b:";
    _tmpStr += c00.b;
    _tmpStr += " g:";
    _tmpStr += c00.g;

    // _tmpStr += " r:";
    // _tmpStr += r;
    // _tmpStr += " b:";
    // _tmpStr += b;
    // _tmpStr += " g:";
    // _tmpStr += g;
    _tmpStr += " tmpInt:";
    _tmpStr += tmpInt;

    serPrntNL(_tmpStr);
    }
  }

//-----------------------------------------------------------------------------
void paramSetColorHandler(String nCmd, String nParamName, int& nParam, int nVal) {
  paramSetHandler(nCmd, nParamName, nParam, nVal, 255, 0);
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
  paramIncHandler(nCmd, nParamName, nParam, nInc, 255, 0);
  }

//-----------------------------------------------------------------------------
void paramIncColorHandler(String nCmd, String nParamName, float& nParam, int nInc) {
  paramIncHandler(nCmd, nParamName, nParam, nInc, 255, 0);
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

//-----------------------------------------------------------------------------
void taskHandleSerIn() {
  if (recvWithEndMarker() > "") {
    int tmpRmax = eeprom_live[EE_REG_R_MAX];
    int tmpGmax = eeprom_live[EE_REG_G_MAX];
    int tmpBmax = eeprom_live[EE_REG_B_MAX];
    int tmpRmin = eeprom_live[EE_REG_R_MIN];
    int tmpGmin = eeprom_live[EE_REG_G_MIN];
    int tmpBmin = eeprom_live[EE_REG_B_MIN];
    int tmpRint = eeprom_live[EE_REG_R_INT];
    int tmpGint = eeprom_live[EE_REG_G_INT];
    int tmpBint = eeprom_live[EE_REG_B_INT];

    paramIncHandler("nm+", "next neopixel mode", npxlMode, 1, NUM_NPX_MODES, 0);
    paramIncHandler("nm-", "next neopixel mode", npxlMode, -1, NUM_NPX_MODES, 0);

    eeprom_live[EE_REG_NEOPIXEL_MODE] = npxlMode;



    paramIncColorHandler("rR+", " max", tmpRmax, 1);
    paramIncColorHandler("rR-", "  max", tmpRmax, -1);
    paramIncColorHandler("rR++", " max", tmpRmax, 5);
    paramIncColorHandler("rR--", "  max", tmpRmax, -5);
    paramIncHandler("rG+", " max", tmpGmax, 1, 255, 0);
    paramIncHandler("rG-", "  max", tmpGmax, -1, 255, 0);
    paramIncHandler("rG++", " max", tmpGmax, 5, 255, 0);
    paramIncHandler("rG--", "  max", tmpGmax, -5, 255, 0);
    paramIncHandler("rB+", " max", tmpBmax, 1, 255, 0);
    paramIncHandler("rB-", "  max", tmpBmax, -1, 255, 0);
    paramIncHandler("rB++", " max", tmpBmax, 5, 255, 0);
    paramIncHandler("rB--", "  max", tmpBmax, -5, 255, 0);

    paramIncHandler("ir+", " increment", tmpRint, .1, 64, 0);
    paramIncHandler("ir-", "  increment", tmpRint, -.1, 64, 0);
    paramIncHandler("ir++", " increment", tmpRint, .1, 64, 0);
    paramIncHandler("ir--", "  increment", tmpRint, -.1, 64, 0);
    paramIncHandler("ig+", " increment", tmpGint, .1, 64, 0);
    paramIncHandler("ig-", "  increment", tmpGint, -.1, 64, 0);
    paramIncHandler("ig++", " increment", tmpGint, 1, 64, 0);
    paramIncHandler("ig--", "  increment", tmpGint, -1, 64, 0);
    paramIncHandler("ib+", " increment", tmpBint, 1, 64, 0);
    paramIncHandler("ib-", "  increment", tmpBint, -1, 64, 0);
    paramIncHandler("ib++", " increment", tmpBint, 1, 64, 0);
    paramIncHandler("ib--", "  increment", tmpBint, -1, 64, 0);


    paramSetHandler("rrmx", "max ", tmpRmax, 255, 255, 1);
    paramSetHandler("rrmd", "mid ", tmpRmax, 127, 255, 1);
    paramSetHandler("rrmn", "min ", tmpRmax, 1, 255, 1);

    paramSetHandler("rgmx", "max ", tmpGmax, 255, 255, 1);
    paramSetHandler("rgmd", "mid ", tmpGmax, 127, 255, 1);
    paramSetHandler("rgmn", "min ", tmpGmax, 1, 255, 1);

    paramSetHandler("rbmx", "max ", tmpBmax, 255, 255, 1);
    paramSetHandler("rbmd", "mid ", tmpBmax, 127, 255, 1);
    paramSetHandler("rbmn", "min ", tmpBmax, 1, 255, 1);

    serPrntVNL("tmpRmax", tmpRmax);

    eeprom_live[EE_REG_R_MAX] = tmpRmax;
    eeprom_live[EE_REG_G_MAX] = tmpGmax;
    eeprom_live[EE_REG_B_MAX] = tmpBmax;
    eeprom_live[EE_REG_R_MIN] = tmpRmin;
    eeprom_live[EE_REG_G_MIN] = tmpGmin;
    eeprom_live[EE_REG_B_MIN] = tmpBmin;
    eeprom_live[EE_REG_R_INT] = tmpRint;
    eeprom_live[EE_REG_G_INT] = tmpGint;
    eeprom_live[EE_REG_B_INT] = tmpBint;
    // calcColorRange();



    if (inStr == "non") {
      nxplEn = true;
      serPrntNL("on: neopixel ring on");
      }

    else if (inStr == "noff") {
      nxplEn = false;
      serPrntNL("off: neopixel ring off");
      }

    else if (inStr == "n+") {
      npxl_rotation_dir = 2;
      serPrntNL("n+: neopixel rotation CCW");
      }
    else if (inStr == "n-") {
      npxl_rotation_dir = 1;
      serPrntNL("n-: neopixel rotation CCW");
      }

    else if (inStr == "n0") {
      npxl_rotation_dir = 0;
      serPrntNL("n-: neopixel rotation stop");

      }
    else if (inStr == "save") {
      serPrntNL("save: save eeprom");
      writeEEPROM();
      btnStateG = true;
      serPrntNL("save: eeprom saved");

      }
    else if (inStr == "read") {
      serPrntNL("read: read eeprom");
      readEEPROM();
      // calcColorRange();
      btnState = true;
      serPrntNL("read: eeprom read");

      }
    else if (inStr == "prnt") {
      serPrntNL("prnt: read eeprom");
      serPrintLiveEEPROM();

      }
    else if (inStr == "init") {
      serPrntNL("init: init eeprom values");

      eeprom_live[EE_REG_NEOPIXEL_MODE] = NPX_MD_ASYC_SINE;
      eeprom_live[EE_REG_R_MAX] = DEF_NPX_AMP_MAX;
      eeprom_live[EE_REG_G_MAX] = DEF_NPX_AMP_MAX;
      eeprom_live[EE_REG_B_MAX] = DEF_NPX_AMP_MAX;
      eeprom_live[EE_REG_R_MIN] = DEF_NPX_AMP_MIN;
      eeprom_live[EE_REG_G_MIN] = DEF_NPX_AMP_MIN;
      eeprom_live[EE_REG_B_MIN] = DEF_NPX_AMP_MIN;
      eeprom_live[EE_REG_R_INT] = 1;
      eeprom_live[EE_REG_G_INT] = 2;
      eeprom_live[EE_REG_B_INT] = 3;
      // calcColorRange();
      serPrintLiveEEPROM();
      }
    else if (inStr == "btn") {
      serPrntNL("btn: sim button");
      btnState = true;
      }
    else if (inStr == "btnR") {
      serPrntNL("btnR: sim button");
      btnStateR = true;
      }
    else if (inStr == "btnG") {
      serPrntNL("btnG: sim button");
      btnStateG = true;
      }
    else if (inStr == "btnB") {
      serPrntNL("btnB: sim button");
      btnStateB = true;
      }
    else if (inStr == "btnRG") {
      serPrntNL("btnRG: sim button");
      btnStateRG = true;
      }

    inStr = "";
    }
  }

//=================================================================================================
void setAllNeoPixels() {
  for (int i = 0; i < NUM_NEOPIXELS; i++)
    strip.setPixelColor(i, c00.r, c00.g, c00.b);
  strip.show();
  }


//=================================================================================================
void taskNeopixelRing() {
  switch (npxlMode) {
      default:
      case NPX_MD_OFF:
        c00.r = 0;
        c00.g = 0;
        c00.b = 0;
        break;

      case NPX_MD_ASYC_SINE:
        c00.r = npxR.npcLedSine(incR, eeprom_live[EE_REG_R_MAX]);
        c00.g = npxG.npcLedSine(incG, eeprom_live[EE_REG_G_MAX]);
        c00.b = npxB.npcLedSine(incB, eeprom_live[EE_REG_B_MAX]);

        break;

      case NPX_MD_RED_SINE:
        c00.r = npxR.npcLedSine(incR, eeprom_live[EE_REG_R_MAX]);
        c00.g = 0;
        c00.b = 0;
        break;

      case NPX_MD_GREEN_SINE:
        c00.r = 0;
        c00.g = npxG.npcLedSine(incG, eeprom_live[EE_REG_G_MAX]);
        c00.b = 0;
        break;

      case NPX_MD_BLUE_SINE:
        c00.r = 0;
        c00.g = 0;
        c00.b = npxB.npcLedSine(incB, eeprom_live[EE_REG_B_MAX]);
        break;

      case NPX_MD_STATIC:
        c00.r = eeprom_live[EE_REG_R_MAX];
        c00.g = eeprom_live[EE_REG_G_MAX];
        c00.b = eeprom_live[EE_REG_B_MAX];
        break;

      case NPX_MD_STATIC_RED:
        c00.r = eeprom_live[EE_REG_B_MAX];
        c00.g = 0;
        c00.b = 0;
        break;

      case NPX_MD_STATIC_GREEN:
        c00.r = 0;
        c00.g = eeprom_live[EE_REG_G_MAX];
        c00.b = 0;
        break;

      case NPX_MD_STATIC_BLUE:
        c00.r = 0;
        c00.g = 0;
        c00.b = eeprom_live[EE_REG_B_MAX];
        break;

    }

  strip.clear();
  // rTmpFloat = 10;
  // rTmpFloat *= i/16.0;
  // rTmpFloat *= 2 * PI;
  // rTmpFloat = sin(rTmpFloat);

  nre0.nreBtnOvr(&c00);
  nreR.nreBtnOvr(&c00);
  nreG.nreBtnOvr(&c00);
  nreB.nreBtnOvr(&c00);
  nreRG.nreBtnOvr(&c00);

  setAllNeoPixels();

  }

//=================================================================================================
void loop() {
  taskHandleSerIn();

  if (iCount == 15) {
    dir = 1;
    }

  if (iCount % 1 == 0 and iCount > 15)
    taskNeopixelRing();

  taskSerOut();

  iCount++;
  delay(10);
  }