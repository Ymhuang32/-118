#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// =====================================================
// BLE UART UUID
// LightBlue 連線後，寫入 RX characteristic
// =====================================================
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

volatile bool bleVLSCommand = false;
volatile bool bleEBSCommand = false;
volatile bool bleResetCommand = false;
volatile bool bleStatusCommand = false;


// =====================================================
// 黑色判定
// 如果白色上顯示 11111、黑線上顯示 00000，就把 0 改成 1
// =====================================================
const int BLACK_VALUE = 0;


// =======================
// 感測器腳位
// =======================
const int S1 = 33;  // 最左
const int S2 = 22;
const int S3 = 34;  // 中間
const int S4 = 35;
const int S5 = 36;  // 最右


// =======================
// 左輪 L298N
// =======================
const int LEFT_PWM = 18;
const int LEFT1 = 14;
const int LEFT2 = 27;


// =======================
// 右輪 L298N
// =======================
const int RIGHT_PWM = 19;
const int RIGHT1 = 26;
const int RIGHT2 = 23;


// =====================================================
// ASL 狀態燈腳位
// 接法：GPIO -> 電阻 220Ω~330Ω -> LED -> GND
// =====================================================
const int ASL_GREEN = 13;
const int ASL_RED   = 25;
const int ASL_BLUE  = 17;


// =====================================================
// 系統狀態
// =====================================================
const int SYS_SAFE = 0;       // 綠燈，待命安全狀態
const int SYS_COUNTDOWN = 1;  // 藍燈，VLS 觸發後倒數
const int SYS_AUTO = 2;       // 紅燈，自主循跡
const int SYS_EBS = 3;        // 藍燈，緊急停止

int systemState = SYS_SAFE;

const unsigned long VLS_START_DELAY = 3000;  // VLS 後 3 秒起跑，小於 5 秒
unsigned long vlsStartTime = 0;

String ebsReason = "";


// =====================================================
// 軟體失控保護
// 如果很久都找不到黑線，當作軟體異常，觸發 EBS
// =====================================================
unsigned long noLineSince = 0;
const unsigned long SOFTWARE_EBS_TIMEOUT = 3500;


// =====================================================
// 循跡模式
// =====================================================
const int MODE_NORMAL = 1;
const int MODE_ARC_LEFT = 2;
const int MODE_ARC_RIGHT = 3;
const int MODE_SPIN_LEFT = 4;
const int MODE_SPIN_RIGHT = 5;

int mode = MODE_NORMAL;


// =====================================================
// 直線校正
// 左輪之前比較慢，所以左輪補高
// =====================================================
int leftStraight  = 148;
int rightStraight = 129;


// =====================================================
// 一般循跡速度：極限測試
// =====================================================
int straightBaseSpeed = 132;

int maxPWM = 220;

int minLeftMovingSpeed  = 98;
int minRightMovingSpeed = 100;

int minLeftReverseSpeed  = 92;
int minRightReverseSpeed = 92;

int minCurveBaseSpeed = 100;


// =====================================================
// 進彎減速設定
// =====================================================
const unsigned long curveSlowHoldTime = 170;
unsigned long curveSlowUntil = 0;

int slowStraightBaseSpeed = 112;
int slowMinCurveBaseSpeed = 94;


// =====================================================
// PD 微調參數：高速防晃版
// =====================================================
float Kp = 8.0;
float Kd = 12.5;

float filterAlpha = 0.70;
float deadBand = 0.55;

int maxCorrectionNormal = 28;
int maxCorrectionSlow = 24;


// =====================================================
// 漏斗形 / 大彎：弧形補線
// =====================================================
int arcOuterSpeed = 178;
int arcInnerSpeed = 98;

int slowArcOuterSpeed = 158;
int slowArcInnerSpeed = 86;

const unsigned long minArcTime = 45;
const unsigned long maxArcTime = 390;

unsigned long arcStartTime = 0;


// =====================================================
// 直角彎：反轉恢復
// =====================================================
int spinOuterSpeed = 182;
int spinReverseSpeed = 122;

int slowSpinOuterSpeed = 164;
int slowSpinReverseSpeed = 112;

const unsigned long minSpinTime = 60;
const unsigned long maxSpinTime = 680;

unsigned long spinStartTime = 0;


// =====================================================
// 全白確認
// =====================================================
unsigned long lostLineStartTime = 0;
const unsigned long lostLineConfirmTime = 20;


// =====================================================
// 短期方向歷史
// =====================================================
const int HISTORY_SIZE = 8;

int historyScore[HISTORY_SIZE];
bool historyStrongEdge[HISTORY_SIZE];
unsigned long historyTime[HISTORY_SIZE];

int historyIndex = 0;

const unsigned long directionHistoryWindow = 220;
const unsigned long strongEdgeWindow = 180;


// =====================================================
// 完全沒有方向資訊時的預設方向
// 1 = 右，-1 = 左
// =====================================================
int defaultRecoverDirection = 1;


// =====================================================
// 特殊狀況
// =====================================================
int allBlackSpeed = 108;
int wideCenterSpeed = 112;


// =====================================================
// 邊緣大修正
// =====================================================
int edgeOuterSpeed = 176;
int edgeInnerSpeed = 96;

int slowEdgeOuterSpeed = 158;
int slowEdgeInnerSpeed = 86;


// =====================================================
// 起步補償
// =====================================================
bool leftWasMoving = false;
bool rightWasMoving = false;

int leftKickSpeed  = 185;
int leftKickTime   = 7;

int rightKickSpeed = 185;
int rightKickTime  = 7;


// =====================================================
// 平滑輸出
// =====================================================
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

int accelerationStep = 58;


// =====================================================
// PD 狀態
// =====================================================
float filteredOffset = 0;
float lastOffset = 0;


// =====================================================
// Debug
// =====================================================
unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 300;


// =====================================================
// BLE Server callback
// =====================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};


// =====================================================
// BLE RX callback
// 手機 LightBlue 寫入文字後，這裡接收
// =====================================================
class MyRXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String cmd = String(pCharacteristic->getValue().c_str());

    cmd.trim();

    if (cmd == "1") {
      bleVLSCommand = true;
    }
    else if (cmd == "0") {
      bleEBSCommand = true;
    }
  }
};


// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);

  pinMode(ASL_GREEN, OUTPUT);
  pinMode(ASL_RED, OUTPUT);
  pinMode(ASL_BLUE, OUTPUT);

  ledcAttach(LEFT_PWM, 1000, 8);
  ledcAttach(RIGHT_PWM, 1000, 8);

  clearHistory();
  stopMotor();

  systemState = SYS_SAFE;
  updateASL();

  setupBLE();

  Serial.println("LINE FOLLOWER WITH BLE VLS + BLE EBS + ASL START");
}


// =====================================================
// loop
// =====================================================
void loop() {
  int v1 = readBlack(S1);
  int v2 = readBlack(S2);
  int v3 = readBlack(S3);
  int v4 = readBlack(S4);
  int v5 = readBlack(S5);

  int count = v1 + v2 + v3 + v4 + v5;

  unsigned long now = millis();

  int targetL = 0;
  int targetR = 0;
  String action = "";

  handleBLEConnection();
  handleBLECommands(now);


  // =====================================================
  // 1. EBS 狀態
  // 馬達保持停止，藍燈亮
  // 手機傳 RESET 回安全狀態，或傳 VLS 重新倒數起跑
  // =====================================================
  if (systemState == SYS_EBS) {
    stopMotor();
    updateASL();

    action = "BLE EBS STOPPED";
    debugPrint(v1, v2, v3, v4, v5, count, 0, 0, 0, 0, action);

    delay(10);
    return;
  }


  // =====================================================
  // 2. SAFE 待命狀態
  // 綠燈亮，馬達不動
  // 手機傳 VLS 後進入倒數
  // =====================================================
  if (systemState == SYS_SAFE) {
    stopMotor();
    updateASL();

    action = "SAFE WAITING FOR BLE VLS";
    debugPrint(v1, v2, v3, v4, v5, count, 0, 0, 0, 0, action);

    delay(10);
    return;
  }


  // =====================================================
  // 3. VLS 倒數狀態
  // 藍燈亮，馬達停止
  // 3 秒後進入自主循跡
  // =====================================================
  if (systemState == SYS_COUNTDOWN) {
    stopMotor();
    updateASL();

    if (now - vlsStartTime >= VLS_START_DELAY) {
      systemState = SYS_AUTO;
      resetLineFollowState();

      action = "BLE VLS COUNTDOWN FINISHED, AUTO START";
      sendBLEMessage("AUTO START");
    }
    else {
      action = "BLE VLS COUNTDOWN";
    }

    debugPrint(v1, v2, v3, v4, v5, count, 0, 0, 0, 0, action);

    delay(10);
    return;
  }


  // =====================================================
  // 4. AUTO 狀態
  // 紅燈亮，開始循跡
  // =====================================================
  updateASL();


  // =====================================================
  // 5. 軟體失控保護
  // 如果長時間完全沒有看到黑線，觸發 EBS
  // =====================================================
  if (count == 0) {
    if (noLineSince == 0) {
      noLineSince = now;
    }

    if (now - noLineSince > SOFTWARE_EBS_TIMEOUT) {
      triggerEBS("SOFTWARE LOST LINE TIMEOUT");
      debugPrint(v1, v2, v3, v4, v5, count, 0, 0, 0, 0, "SOFTWARE EBS LOST LINE");
      delay(10);
      return;
    }
  }
  else {
    noLineSince = 0;
  }


  bool curveSlow = now < curveSlowUntil;


  // =====================================================
  // B. 漏斗形左彎：弧形補線
  // =====================================================
  if (mode == MODE_ARC_LEFT) {
    unsigned long elapsed = now - arcStartTime;

    int useOuter = curveSlow ? slowArcOuterSpeed : arcOuterSpeed;
    int useInner = curveSlow ? slowArcInnerSpeed : arcInnerSpeed;

    bool lineBack = false;

    if (count > 0) {
      if (v1 == 1 || v2 == 1 || v3 == 1) {
        lineBack = true;
      }
    }

    if (elapsed > minArcTime && lineBack) {
      mode = MODE_NORMAL;
      filteredOffset = 0;
      lastOffset = 0;
      lostLineStartTime = 0;

      targetL = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
      targetR = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
      action = "EXIT ARC LEFT";
    }
    else if (elapsed > maxArcTime) {
      mode = MODE_SPIN_LEFT;
      spinStartTime = now;

      targetL = -(curveSlow ? slowSpinReverseSpeed : spinReverseSpeed);
      targetR = curveSlow ? slowSpinOuterSpeed : spinOuterSpeed;
      action = "ARC LEFT FAIL ENTER SPIN LEFT";
    }
    else {
      targetL = useInner;
      targetR = useOuter;
      action = "ARC LEFT FUNNEL RECOVER";
    }

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, getHistorySum(now), 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // C. 漏斗形右彎：弧形補線
  // =====================================================
  if (mode == MODE_ARC_RIGHT) {
    unsigned long elapsed = now - arcStartTime;

    int useOuter = curveSlow ? slowArcOuterSpeed : arcOuterSpeed;
    int useInner = curveSlow ? slowArcInnerSpeed : arcInnerSpeed;

    bool lineBack = false;

    if (count > 0) {
      if (v3 == 1 || v4 == 1 || v5 == 1) {
        lineBack = true;
      }
    }

    if (elapsed > minArcTime && lineBack) {
      mode = MODE_NORMAL;
      filteredOffset = 0;
      lastOffset = 0;
      lostLineStartTime = 0;

      targetL = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
      targetR = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
      action = "EXIT ARC RIGHT";
    }
    else if (elapsed > maxArcTime) {
      mode = MODE_SPIN_RIGHT;
      spinStartTime = now;

      targetL = curveSlow ? slowSpinOuterSpeed : spinOuterSpeed;
      targetR = -(curveSlow ? slowSpinReverseSpeed : spinReverseSpeed);
      action = "ARC RIGHT FAIL ENTER SPIN RIGHT";
    }
    else {
      targetL = useOuter;
      targetR = useInner;
      action = "ARC RIGHT FUNNEL RECOVER";
    }

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, getHistorySum(now), 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // D. 左直角彎：反轉恢復
  // =====================================================
  if (mode == MODE_SPIN_LEFT) {
    unsigned long elapsed = now - spinStartTime;

    int useOuter = curveSlow ? slowSpinOuterSpeed : spinOuterSpeed;
    int useReverse = curveSlow ? slowSpinReverseSpeed : spinReverseSpeed;

    bool lineBack = false;

    if (elapsed > minSpinTime) {
      if (v3 == 1 || (v2 == 1 && v1 == 0) || (v2 == 1 && v3 == 1)) {
        lineBack = true;
      }
    }

    if (lineBack) {
      mode = MODE_NORMAL;
      filteredOffset = 0;
      lastOffset = 0;
      lostLineStartTime = 0;
      clearHistory();

      targetL = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
      targetR = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
      action = "EXIT SPIN LEFT";
    }
    else if (elapsed > maxSpinTime) {
      mode = MODE_SPIN_RIGHT;
      spinStartTime = now;

      targetL = useOuter;
      targetR = -useReverse;
      action = "SPIN LEFT TIMEOUT SWITCH RIGHT";
    }
    else {
      targetL = -useReverse;
      targetR = useOuter;
      action = "SPIN LEFT SHARP RECOVER";
    }

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, getHistorySum(now), 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // E. 右直角彎：反轉恢復
  // =====================================================
  if (mode == MODE_SPIN_RIGHT) {
    unsigned long elapsed = now - spinStartTime;

    int useOuter = curveSlow ? slowSpinOuterSpeed : spinOuterSpeed;
    int useReverse = curveSlow ? slowSpinReverseSpeed : spinReverseSpeed;

    bool lineBack = false;

    if (elapsed > minSpinTime) {
      if (v3 == 1 || (v4 == 1 && v5 == 0) || (v3 == 1 && v4 == 1)) {
        lineBack = true;
      }
    }

    if (lineBack) {
      mode = MODE_NORMAL;
      filteredOffset = 0;
      lastOffset = 0;
      lostLineStartTime = 0;
      clearHistory();

      targetL = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
      targetR = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
      action = "EXIT SPIN RIGHT";
    }
    else if (elapsed > maxSpinTime) {
      mode = MODE_SPIN_LEFT;
      spinStartTime = now;

      targetL = -useReverse;
      targetR = useOuter;
      action = "SPIN RIGHT TIMEOUT SWITCH LEFT";
    }
    else {
      targetL = useOuter;
      targetR = -useReverse;
      action = "SPIN RIGHT SHARP RECOVER";
    }

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, getHistorySum(now), 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // F. NORMAL：全白 00000
  // =====================================================
  if (count == 0) {
    if (lostLineStartTime == 0) {
      lostLineStartTime = now;
    }

    if (now - lostLineStartTime < lostLineConfirmTime) {
      targetL = currentLeftSpeed;
      targetR = currentRightSpeed;
      action = "WHITE CONFIRMING";
    }
    else {
      curveSlowUntil = now + curveSlowHoldTime;

      int direction = chooseRecoverDirection(now);
      bool strongRecently = hasRecentStrongEdge(now);

      if (strongRecently) {
        if (direction < 0) {
          mode = MODE_SPIN_LEFT;
          spinStartTime = now;

          targetL = -slowSpinReverseSpeed;
          targetR = slowSpinOuterSpeed;
          action = "WHITE ENTER SLOW SPIN LEFT SHARP";
        }
        else {
          mode = MODE_SPIN_RIGHT;
          spinStartTime = now;

          targetL = slowSpinOuterSpeed;
          targetR = -slowSpinReverseSpeed;
          action = "WHITE ENTER SLOW SPIN RIGHT SHARP";
        }
      }
      else {
        if (direction < 0) {
          mode = MODE_ARC_LEFT;
          arcStartTime = now;

          targetL = slowArcInnerSpeed;
          targetR = slowArcOuterSpeed;
          action = "WHITE ENTER SLOW ARC LEFT FUNNEL";
        }
        else {
          mode = MODE_ARC_RIGHT;
          arcStartTime = now;

          targetL = slowArcOuterSpeed;
          targetR = slowArcInnerSpeed;
          action = "WHITE ENTER SLOW ARC RIGHT FUNNEL";
        }
      }
    }

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, getHistorySum(now), 0, targetL, targetR, action);
    delay(3);
    return;
  }
  else {
    lostLineStartTime = 0;
  }


  // =====================================================
  // G. 計算位置
  // =====================================================
  int weightedSum = 0;
  weightedSum += v1 * (-4);
  weightedSum += v2 * (-2);
  weightedSum += v3 * 0;
  weightedSum += v4 * 2;
  weightedSum += v5 * 4;

  float rawOffset = (float)weightedSum / count;


  // =====================================================
  // H. 記錄短期方向歷史
  // =====================================================
  recordDirectionHistory(v1, v2, v3, v4, v5, now);


  // =====================================================
  // I. 偵測彎道徵兆，開啟進彎減速
  // =====================================================
  bool curveSignal = false;

  if (v1 == 1 || v5 == 1) {
    curveSignal = true;
  }

  if (fabs(rawOffset) >= 1.35) {
    curveSignal = true;
  }

  if (v3 == 0 && (v2 == 1 || v4 == 1)) {
    curveSignal = true;
  }

  if (curveSignal) {
    curveSlowUntil = now + curveSlowHoldTime;
  }

  curveSlow = now < curveSlowUntil;


  // =====================================================
  // J. 全黑 11111
  // =====================================================
  if (count == 5) {
    filteredOffset = 0;
    lastOffset = 0;

    targetL = allBlackSpeed + (leftStraight - straightBaseSpeed);
    targetR = allBlackSpeed + (rightStraight - straightBaseSpeed);
    action = "ALL BLACK SLOW STRAIGHT";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // K. 中間寬線 01110
  // =====================================================
  if (v1 == 0 && v2 == 1 && v3 == 1 && v4 == 1 && v5 == 0) {
    filteredOffset = 0;
    lastOffset = 0;

    targetL = wideCenterSpeed + (leftStraight - straightBaseSpeed);
    targetR = wideCenterSpeed + (rightStraight - straightBaseSpeed);
    action = "WIDE BLACK CENTER STRAIGHT";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // L. 正中間 00100
  // 高速直線：直接走，不做 PD 左右修
  // =====================================================
  if (v1 == 0 && v2 == 0 && v3 == 1 && v4 == 0 && v5 == 0) {
    filteredOffset = 0;
    lastOffset = 0;

    int baseL = curveSlow ? 122 : leftStraight;
    int baseR = curveSlow ? 106 : rightStraight;

    targetL = baseL;
    targetR = baseR;
    action = curveSlow ? "CENTER STRAIGHT BUT SLOW" : "CENTER BLACK STRAIGHT";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // M. 輕微偏左 / 輕微偏右：溫和修正
  // =====================================================
  if (v1 == 0 && v2 == 1 && v3 == 1 && v4 == 0 && v5 == 0) {
    targetL = curveSlow ? 110 : 130;
    targetR = curveSlow ? 118 : 138;
    action = curveSlow ? "SLOW GENTLE LEFT" : "GENTLE LEFT";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }

  if (v1 == 0 && v2 == 0 && v3 == 1 && v4 == 1 && v5 == 0) {
    targetL = curveSlow ? 132 : 158;
    targetR = curveSlow ? 100 : 118;
    action = curveSlow ? "SLOW GENTLE RIGHT" : "GENTLE RIGHT";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // N. 最外側看到黑：強差速
  // =====================================================
  if (v1 == 1 && v3 == 0 && v5 == 0) {
    targetL = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
    targetR = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
    action = curveSlow ? "SLOW EDGE LEFT FAST ARC" : "EDGE LEFT FAST ARC";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }

  if (v5 == 1 && v3 == 0 && v1 == 0) {
    targetL = curveSlow ? slowEdgeOuterSpeed : edgeOuterSpeed;
    targetR = curveSlow ? slowEdgeInnerSpeed : edgeInnerSpeed;
    action = curveSlow ? "SLOW EDGE RIGHT FAST ARC" : "EDGE RIGHT FAST ARC";

    driveSmooth(targetL, targetR);
    debugPrint(v1, v2, v3, v4, v5, count, rawOffset, 0, targetL, targetR, action);
    delay(3);
    return;
  }


  // =====================================================
  // O. 一般狀況：PD 微調
  // =====================================================
  filteredOffset = filterAlpha * filteredOffset + (1.0 - filterAlpha) * rawOffset;

  float controlOffset = filteredOffset;

  if (controlOffset > -deadBand && controlOffset < deadBand) {
    controlOffset = 0;
  }

  float absOffset = fabs(controlOffset);

  int baseSpeed = curveSlow ? slowStraightBaseSpeed : straightBaseSpeed;

  baseSpeed = baseSpeed - (int)(absOffset * 6);

  if (count == 2) {
    baseSpeed -= 1;
  }
  else if (count == 3) {
    baseSpeed -= 3;
  }
  else if (count >= 4) {
    baseSpeed -= 6;
  }

  int useMinBase = curveSlow ? slowMinCurveBaseSpeed : minCurveBaseSpeed;

  if (baseSpeed < useMinBase) {
    baseSpeed = useMinBase;
  }

  float derivative = controlOffset - lastOffset;
  lastOffset = controlOffset;

  int correction = (int)(Kp * controlOffset + Kd * derivative);

  int limitCorrection = curveSlow ? maxCorrectionSlow : maxCorrectionNormal;
  correction = clampInt(correction, -limitCorrection, limitCorrection);

  if (v1 == 1) {
    correction -= 9;
    baseSpeed -= 3;
    action = curveSlow ? "SLOW EDGE LEFT NORMAL CORRECT" : "EDGE LEFT NORMAL CORRECT";
  }
  else if (v5 == 1) {
    correction += 9;
    baseSpeed -= 3;
    action = curveSlow ? "SLOW EDGE RIGHT NORMAL CORRECT" : "EDGE RIGHT NORMAL CORRECT";
  }
  else if (controlOffset < 0) {
    action = curveSlow ? "SLOW LEFT MICRO CORRECT" : "LEFT MICRO CORRECT";
  }
  else if (controlOffset > 0) {
    action = curveSlow ? "SLOW RIGHT MICRO CORRECT" : "RIGHT MICRO CORRECT";
  }
  else {
    action = curveSlow ? "SLOW STABLE FORWARD" : "STABLE FORWARD";
  }

  if (baseSpeed < useMinBase) {
    baseSpeed = useMinBase;
  }

  targetL = baseSpeed + correction;
  targetR = baseSpeed - correction;

  targetL = targetL + (leftStraight - straightBaseSpeed);
  targetR = targetR + (rightStraight - straightBaseSpeed);

  driveSmooth(targetL, targetR);

  debugPrint(v1, v2, v3, v4, v5, count, rawOffset, filteredOffset, targetL, targetR, action);

  delay(3);
}


// =====================================================
// BLE 初始化
// =====================================================
void setupBLE() {
  BLEDevice::init("ESP32_LINE_CAR");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pRxCharacteristic->setCallbacks(new MyRXCallbacks());

  pService->start();

  pServer->getAdvertising()->start();

  Serial.println("BLE READY: connect to ESP32_LINE_CAR");
}


// =====================================================
// BLE 連線處理
// =====================================================
void handleBLEConnection() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    BLEDevice::startAdvertising();
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    sendBLEMessage("CONNECTED TO ESP32_LINE_CAR");
  }
}


// =====================================================
// BLE 指令處理
// =====================================================
void handleBLECommands(unsigned long now) {
  if (bleEBSCommand) {
    bleEBSCommand = false;

    triggerEBS("BLE EBS COMMAND");
    sendBLEMessage("EBS TRIGGERED");
  }

  if (bleResetCommand) {
    bleResetCommand = false;

    systemState = SYS_SAFE;
    ebsReason = "";

    resetLineFollowState();
    stopMotor();
    updateASL();

    sendBLEMessage("RESET TO SAFE");
  }

  if (bleVLSCommand) {
    bleVLSCommand = false;

    if (systemState == SYS_SAFE || systemState == SYS_EBS) {
      ebsReason = "";

      systemState = SYS_COUNTDOWN;
      vlsStartTime = now;

      resetLineFollowState();
      stopMotor();
      updateASL();

      sendBLEMessage("VLS RECEIVED, COUNTDOWN 3 SEC");
    }
    else if (systemState == SYS_AUTO) {
      sendBLEMessage("ALREADY AUTO");
    }
    else if (systemState == SYS_COUNTDOWN) {
      sendBLEMessage("ALREADY COUNTDOWN");
    }
  }

  if (bleStatusCommand) {
    bleStatusCommand = false;
    sendStatusMessage();
  }
}


// =====================================================
// BLE 傳訊息回手機
// =====================================================
void sendBLEMessage(String msg) {
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
  }
}


// =====================================================
// BLE 狀態回報
// =====================================================
void sendStatusMessage() {
  String msg = "SYS=";
  msg += systemState;
  msg += " MODE=";
  msg += mode;
  msg += " EBS=";
  msg += ebsReason;

  sendBLEMessage(msg);
}


// =====================================================
// EBS / ASL 功能
// =====================================================
void triggerEBS(String reason) {
  systemState = SYS_EBS;
  ebsReason = reason;

  stopMotor();
  resetLineFollowState();
  updateASL();
}

void updateASL() {
  digitalWrite(ASL_GREEN, LOW);
  digitalWrite(ASL_RED, LOW);
  digitalWrite(ASL_BLUE, LOW);

  if (systemState == SYS_SAFE) {
    digitalWrite(ASL_GREEN, HIGH);
  }
  else if (systemState == SYS_AUTO) {
    digitalWrite(ASL_RED, HIGH);
  }
  else {
    digitalWrite(ASL_BLUE, HIGH);
  }
}

void resetLineFollowState() {
  mode = MODE_NORMAL;

  noLineSince = 0;
  lostLineStartTime = 0;

  curveSlowUntil = 0;

  filteredOffset = 0;
  lastOffset = 0;

  currentLeftSpeed = 0;
  currentRightSpeed = 0;

  leftWasMoving = false;
  rightWasMoving = false;

  clearHistory();
}


// =====================================================
// 記錄短期方向歷史
//
// score > 0：黑線在右
// score < 0：黑線在左
// =====================================================
void recordDirectionHistory(
  int v1,
  int v2,
  int v3,
  int v4,
  int v5,
  unsigned long now
) {
  int score = 0;

  score += v5 * 5;
  score += v4 * 3;
  score -= v2 * 3;
  score -= v1 * 5;

  bool strongEdge = false;

  if (v1 == 1 || v5 == 1) {
    strongEdge = true;
  }

  if (score == 0) {
    return;
  }

  historyScore[historyIndex] = score;
  historyStrongEdge[historyIndex] = strongEdge;
  historyTime[historyIndex] = now;

  historyIndex++;

  if (historyIndex >= HISTORY_SIZE) {
    historyIndex = 0;
  }
}


// =====================================================
// 清掉歷史
// =====================================================
void clearHistory() {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    historyScore[i] = 0;
    historyStrongEdge[i] = false;
    historyTime[i] = 0;
  }

  historyIndex = 0;
}


// =====================================================
// 取得最近方向分數
// =====================================================
int getHistorySum(unsigned long now) {
  int sum = 0;

  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (historyTime[i] == 0) {
      continue;
    }

    if (now - historyTime[i] <= directionHistoryWindow) {
      sum += historyScore[i];
    }
  }

  return sum;
}


// =====================================================
// 是否剛剛有外側強訊號
// =====================================================
bool hasRecentStrongEdge(unsigned long now) {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (historyTime[i] == 0) {
      continue;
    }

    if (now - historyTime[i] <= strongEdgeWindow && historyStrongEdge[i]) {
      return true;
    }
  }

  return false;
}


// =====================================================
// 選擇全白後該往哪邊找線
// =====================================================
int chooseRecoverDirection(unsigned long now) {
  int sum = getHistorySum(now);

  if (sum > 0) {
    return 1;
  }

  if (sum < 0) {
    return -1;
  }

  return defaultRecoverDirection;
}


// =====================================================
// 讀取感測器
// 回傳：黑色 = 1，白色 = 0
// =====================================================
int readBlack(int pin) {
  int value = digitalRead(pin);

  if (value == BLACK_VALUE) {
    return 1;
  }
  else {
    return 0;
  }
}


// =====================================================
// 平滑輸出
// =====================================================
void driveSmooth(int targetL, int targetR) {
  targetL = limitLeftSpeed(targetL);
  targetR = limitRightSpeed(targetR);

  currentLeftSpeed = approach(currentLeftSpeed, targetL, accelerationStep);
  currentRightSpeed = approach(currentRightSpeed, targetR, accelerationStep);

  moveMotor(currentLeftSpeed, currentRightSpeed);
}


// =====================================================
// 速度漸進
// =====================================================
int approach(int current, int target, int step) {
  if (current < target) {
    current += step;

    if (current > target) {
      current = target;
    }
  }
  else if (current > target) {
    current -= step;

    if (current < target) {
      current = target;
    }
  }

  return current;
}


// =====================================================
// 整數限制
// =====================================================
int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) {
    return minValue;
  }

  if (value > maxValue) {
    return maxValue;
  }

  return value;
}


// =====================================================
// 左輪限速
// =====================================================
int limitLeftSpeed(int speedValue) {
  if (speedValue == 0) {
    return 0;
  }

  if (speedValue > 0) {
    if (speedValue < minLeftMovingSpeed) {
      speedValue = minLeftMovingSpeed;
    }

    if (speedValue > maxPWM) {
      speedValue = maxPWM;
    }

    return speedValue;
  }

  int reversePWM = abs(speedValue);

  if (reversePWM < minLeftReverseSpeed) {
    reversePWM = minLeftReverseSpeed;
  }

  if (reversePWM > maxPWM) {
    reversePWM = maxPWM;
  }

  return -reversePWM;
}


// =====================================================
// 右輪限速
// =====================================================
int limitRightSpeed(int speedValue) {
  if (speedValue == 0) {
    return 0;
  }

  if (speedValue > 0) {
    if (speedValue < minRightMovingSpeed) {
      speedValue = minRightMovingSpeed;
    }

    if (speedValue > maxPWM) {
      speedValue = maxPWM;
    }

    return speedValue;
  }

  int reversePWM = abs(speedValue);

  if (reversePWM < minRightReverseSpeed) {
    reversePWM = minRightReverseSpeed;
  }

  if (reversePWM > maxPWM) {
    reversePWM = maxPWM;
  }

  return -reversePWM;
}


// =====================================================
// 馬達控制
// 左右輪前進都是 HIGH / LOW
// =====================================================
void moveMotor(int speedLeft, int speedRight) {
  moveLeftMotor(speedLeft);
  moveRightMotor(speedRight);
}


// =====================================================
// 左輪控制
// 正數 = 前進
// 負數 = 後退
// =====================================================
void moveLeftMotor(int speedValue) {
  int pwm = abs(speedValue);

  if (speedValue > 0) {
    digitalWrite(LEFT1, HIGH);
    digitalWrite(LEFT2, LOW);
  }
  else if (speedValue < 0) {
    digitalWrite(LEFT1, LOW);
    digitalWrite(LEFT2, HIGH);
  }
  else {
    digitalWrite(LEFT1, LOW);
    digitalWrite(LEFT2, LOW);
    ledcWrite(LEFT_PWM, 0);
    leftWasMoving = false;
    return;
  }

  if (!leftWasMoving) {
    ledcWrite(LEFT_PWM, leftKickSpeed);
    delay(leftKickTime);
  }

  leftWasMoving = true;
  ledcWrite(LEFT_PWM, pwm);
}


// =====================================================
// 右輪控制
// 正數 = 前進
// 負數 = 後退
// =====================================================
void moveRightMotor(int speedValue) {
  int pwm = abs(speedValue);

  if (speedValue > 0) {
    digitalWrite(RIGHT1, HIGH);
    digitalWrite(RIGHT2, LOW);
  }
  else if (speedValue < 0) {
    digitalWrite(RIGHT1, LOW);
    digitalWrite(RIGHT2, HIGH);
  }
  else {
    digitalWrite(RIGHT1, LOW);
    digitalWrite(RIGHT2, LOW);
    ledcWrite(RIGHT_PWM, 0);
    rightWasMoving = false;
    return;
  }

  if (!rightWasMoving) {
    ledcWrite(RIGHT_PWM, rightKickSpeed);
    delay(rightKickTime);
  }

  rightWasMoving = true;
  ledcWrite(RIGHT_PWM, pwm);
}


// =====================================================
// 停車
// =====================================================
void stopMotor() {
  digitalWrite(LEFT1, LOW);
  digitalWrite(LEFT2, LOW);
  digitalWrite(RIGHT1, LOW);
  digitalWrite(RIGHT2, LOW);

  ledcWrite(LEFT_PWM, 0);
  ledcWrite(RIGHT_PWM, 0);

  currentLeftSpeed = 0;
  currentRightSpeed = 0;

  leftWasMoving = false;
  rightWasMoving = false;
}


// =====================================================
// Debug
// =====================================================
void debugPrint(
  int v1,
  int v2,
  int v3,
  int v4,
  int v5,
  int count,
  float rawOffset,
  float smoothOffset,
  int L,
  int R,
  String action
) {
  unsigned long now = millis();

  if (now - lastDebugTime < debugInterval) {
    return;
  }

  lastDebugTime = now;

  Serial.print("BLACK_SENSOR=");
  Serial.print(v1);
  Serial.print(v2);
  Serial.print(v3);
  Serial.print(v4);
  Serial.print(v5);

  Serial.print(" count=");
  Serial.print(count);

  Serial.print(" sys=");
  Serial.print(systemState);

  Serial.print(" mode=");
  Serial.print(mode);

  Serial.print(" BLE=");
  Serial.print(deviceConnected);

  Serial.print(" hist=");
  Serial.print(getHistorySum(now));


  Serial.print(" strong=");
  Serial.print(hasRecentStrongEdge(now));

  Serial.print(" slow=");
  Serial.print(now < curveSlowUntil);

  Serial.print(" raw=");
  Serial.print(rawOffset);

  Serial.print(" smooth=");
  Serial.print(smoothOffset);

  Serial.print(" L=");
  Serial.print(L);

  Serial.print(" R=");
  Serial.print(R);

  Serial.print(" curL=");
  Serial.print(currentLeftSpeed);

  Serial.print(" curR=");
  Serial.print(currentRightSpeed);

  Serial.print(" EBS=");
  Serial.print(ebsReason);

  Serial.print(" action=");
  Serial.println(action);
}