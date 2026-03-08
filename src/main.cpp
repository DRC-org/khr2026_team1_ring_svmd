#include <Arduino.h>
#include <FastLED_NeoPixel.h>
#include <SPI.h>
#include <Servo.h>
#include <mcp_can.h>

// =====================================================================
// ピン定義
// =====================================================================
#define INT 3
#define SV0 4   // 位置サーボ 0
#define SV1 5   // リングハンド開閉
#define SV2 6   // 位置サーボ 1
#define SV3 7   // 櫓ハンド
#define SV4 8   // タッチセンサー入力（把持失敗検出）
#define RGB 9
#define CS  10

// =====================================================================
// CAN 設定（書き込む基板に合わせて変更: 前=0 / 後=1）
// =====================================================================
#define BOARD_CAN_ID 0

const unsigned int CAN_ID  = 0x400 + BOARD_CAN_ID;
const unsigned char FB_BASE = 0x40 + BOARD_CAN_ID * 0x0A;

// フィードバック識別子
// FB_BASE+0x00: 開閉完了
// FB_BASE+0x01: 位置完了
// FB_BASE+0x02: 把持失敗
// FB_BASE+0x03: 櫓ハンド完了

// =====================================================================
// サーボ角度定数
// =====================================================================

// 位置サーボ目標角度
const float POS_PICKUP_SV0  = 199.0f;
const float POS_PICKUP_SV2  = 264.0f;
const float POS_YAGURA_SV0  = 30.0f;
const float POS_YAGURA_SV2  = 60.0f;
const float POS_HONMARU_SV0 = 30.0f;
const float POS_HONMARU_SV2 = 74.0f;

// リングハンド角度
const float HAND_OPEN_ANGLE  = 120.0f;
const float HAND_CLOSE_ANGLE = 180.0f;

// 櫓ハンド角度
const float YAGURA_OPEN_ANGLE  = 0.0f;
const float YAGURA_CLOSE_ANGLE = 70.0f;

// =====================================================================
// モーション定数
// =====================================================================
const float    SCALE_270         = 180.0f / 270.0f;  // 270°サーボ → servo.write() 変換係数
const float    POS_SERVO_SPEED   = 45.0f;   // 位置サーボ 平均角速度 [deg/s]
const float    YAGURA_SERVO_SPEED = 20.0f;  // 櫓ハンド 平均角速度 [deg/s]
const uint32_t UPDATE_INTERVAL_MS = 10;     // 制御ループ周期 [ms]（100Hz）

// =====================================================================
// 状態定義
// =====================================================================
enum PosState : int8_t {
  POS_PICKUP      = 0,
  POS_YAGURA      = 1,
  POS_HONMARU     = 2,
  POS_STOPPED     = 3,
  POS_PICKUP_DONE = 4,
  POS_YAGURA_DONE = 5,
  POS_HONMARU_DONE = 6,
};

enum HandState : int8_t {
  HAND_OPEN      = 0,
  HAND_CLOSE     = 1,
  HAND_STOPPED   = 2,
  HAND_OPEN_DONE = 3,
  HAND_CLOSE_DONE = 4,
};

// =====================================================================
// グローバル変数
// =====================================================================
Servo servo0;  // SV0: 位置サーボ 0
Servo servo1;  // SV1: リングハンド開閉
Servo servo2;  // SV2: 位置サーボ 1
Servo servo3;  // SV3: 櫓ハンド

FastLED_NeoPixel<1, RGB, NEO_GRB> strip;
MCP_CAN CAN(CS);

// 位置サーボ: servos[0]=SV0, servos[1]=SV2
// リングハンド: servos[2]=SV1
Servo* servos[3] = {&servo0, &servo2, &servo1};

// 位置サーボ（SV0/SV2）モーション
float    currentAngles[3]   = {80.0f, 80.0f, 80.0f};  // [0]=SV0, [1]=SV2, [2]=SV1
float    targetAngles[3]    = {80.0f, 80.0f, 80.0f};
float    startAngles[2]     = {80.0f, 80.0f};
uint32_t motionStartTime[2] = {0, 0};
uint32_t motionDuration[2]  = {0, 0};
PosState pos_state = POS_PICKUP;

// リングハンド（SV1）モーション
float    startAngleHand     = 80.0f;
uint32_t motionStartTimeHand = 0;
uint32_t motionDurationHand  = 0;
HandState hand_state = HAND_OPEN;
bool gripFailSent = false;

// 櫓ハンド（SV3）モーション
float    currentSv3Angle    = 0.0f;
float    targetSv3Angle     = 0.0f;
float    startSv3Angle      = 0.0f;
uint32_t motionStartTimeSv3 = 0;
uint32_t motionDurationSv3  = 0;
HandState sv3_state = HAND_OPEN;

uint32_t lastUpdateTime = 0;

// =====================================================================
// 関数プロトタイプ
// =====================================================================
void updatePosServos();
void updateHandServo();
void updateYaguraHandServo();
void startPosMotion();

// =====================================================================
// サイン波イージングでモーション開始
// =====================================================================
void startPosMotion() {
  uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    startAngles[i]     = currentAngles[i];
    float dist         = fabsf(targetAngles[i] - currentAngles[i]);
    motionDuration[i]  = (uint32_t)(dist / POS_SERVO_SPEED * 1000.0f);
    motionStartTime[i] = now;
  }
}

// =====================================================================
// CAN送信ヘルパー
// =====================================================================
void sendFeedback(unsigned char id, unsigned char param) {
  unsigned char txBuf[8] = {0x00};
  txBuf[0] = id;
  txBuf[1] = param;
  CAN.sendMsgBuf(0x000, 0, 8, txBuf);
}

// =====================================================================
// setup
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(INT, INPUT);
  pinMode(SV0, OUTPUT);
  pinMode(SV1, OUTPUT);
  pinMode(SV2, OUTPUT);
  pinMode(SV3, OUTPUT);
  pinMode(SV4, INPUT_PULLUP);
  pinMode(RGB, OUTPUT);
  pinMode(CS,  OUTPUT);

  strip.begin();
  strip.setBrightness(100);
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();

  servo0.attach(SV0, 500, 2400);
  servo1.attach(SV1, 500, 2400);
  servo2.attach(SV2, 500, 2400);
  servo3.attach(SV3, 500, 2400);

  servo0.write((int)roundf(90.0f * SCALE_270));
  servo1.write(150);
  servo2.write((int)roundf(90.0f * SCALE_270));
  servo3.write(0);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      strip.show();
      delay(100);
      strip.setPixelColor(0, strip.Color(0, 0, 0));
      strip.show();
      delay(100);
    }
  }

  CAN.setMode(MCP_NORMAL);
}

// =====================================================================
// loop
// =====================================================================
void loop() {
  // CAN 受信
  if (!digitalRead(INT)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == CAN_ID) {
      unsigned char command = rxBuf[0];
      unsigned char param   = rxBuf[1];

      if (command == 0x00) {  // 位置移動
        if (param == 0x00) {
          targetAngles[0] = POS_PICKUP_SV0;
          targetAngles[1] = POS_PICKUP_SV2;
          pos_state = POS_PICKUP;
        } else if (param == 0x01) {
          targetAngles[0] = POS_YAGURA_SV0;
          targetAngles[1] = POS_YAGURA_SV2;
          pos_state = POS_YAGURA;
        } else if (param == 0x02) {
          targetAngles[0] = POS_HONMARU_SV0;
          targetAngles[1] = POS_HONMARU_SV2;
          pos_state = POS_HONMARU;
        }
        startPosMotion();

      } else if (command == 0x01) {  // 位置停止
        targetAngles[0] = currentAngles[0];
        targetAngles[1] = currentAngles[1];
        pos_state = POS_STOPPED;

      } else if (command == 0x10) {  // リングハンド 閉じる
        targetAngles[2]  = HAND_CLOSE_ANGLE;
        hand_state       = HAND_CLOSE;
        startAngleHand   = currentAngles[2];
        motionDurationHand = (uint32_t)(fabsf(HAND_CLOSE_ANGLE - currentAngles[2]) / POS_SERVO_SPEED * 1000.0f);
        motionStartTimeHand = millis();

      } else if (command == 0x11) {  // リングハンド 開く
        targetAngles[2]  = HAND_OPEN_ANGLE;
        hand_state       = HAND_OPEN;
        startAngleHand   = currentAngles[2];
        motionDurationHand = (uint32_t)(fabsf(HAND_OPEN_ANGLE - currentAngles[2]) / POS_SERVO_SPEED * 1000.0f);
        motionStartTimeHand = millis();

      } else if (command == 0x12) {  // リングハンド 停止
        targetAngles[2] = currentAngles[2];
        hand_state = HAND_STOPPED;

      } else if (command == 0x20) {  // 櫓ハンド 閉じる
        targetSv3Angle     = YAGURA_CLOSE_ANGLE;
        sv3_state          = HAND_CLOSE;
        startSv3Angle      = currentSv3Angle;
        motionDurationSv3  = (uint32_t)(fabsf(YAGURA_CLOSE_ANGLE - currentSv3Angle) / YAGURA_SERVO_SPEED * 1000.0f);
        motionStartTimeSv3 = millis();

      } else if (command == 0x21) {  // 櫓ハンド 開く
        targetSv3Angle     = YAGURA_OPEN_ANGLE;
        sv3_state          = HAND_OPEN;
        startSv3Angle      = currentSv3Angle;
        motionDurationSv3  = (uint32_t)(fabsf(YAGURA_OPEN_ANGLE - currentSv3Angle) / YAGURA_SERVO_SPEED * 1000.0f);
        motionStartTimeSv3 = millis();

      } else if (command == 0x22) {  // 櫓ハンド 停止
        targetSv3Angle = currentSv3Angle;
        sv3_state = HAND_STOPPED;
      }
    }
  }

  // 制御ループ（100Hz）
  uint32_t now = millis();
  if (now - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    lastUpdateTime = now;
    updatePosServos();
    updateHandServo();
    updateYaguraHandServo();

    // 把持失敗検出: CLOSE_DONE後にタッチセンサーが導通（LOW）したら1回送信
    if (hand_state == HAND_CLOSE_DONE && !gripFailSent && !digitalRead(SV4)) {
      sendFeedback(FB_BASE + 0x02, 0x00);
      gripFailSent = true;
    }
  }
}

// =====================================================================
// 位置サーボ更新（SV0 / SV2）
// =====================================================================
void updatePosServos() {
  uint32_t now = millis();

  // 完了・停止中は現在角を維持
  if (pos_state >= POS_STOPPED) {
    for (int i = 0; i < 2; i++) {
      targetAngles[i] = currentAngles[i];
      servos[i]->write((int)roundf(currentAngles[i] * SCALE_270));
    }
    return;
  }

  // 移動中: SV0/SV2 両方をイージングで更新
  bool all_done = true;
  for (int i = 0; i < 2; i++) {
    uint32_t elapsed = now - motionStartTime[i];
    if (elapsed >= motionDuration[i]) {
      currentAngles[i] = targetAngles[i];
    } else {
      all_done = false;
      float t    = (float)elapsed / (float)motionDuration[i];
      float ease = (1.0f - cosf(PI * t)) / 2.0f;
      currentAngles[i] = startAngles[i] + (targetAngles[i] - startAngles[i]) * ease;
    }
    servos[i]->write((int)roundf(currentAngles[i] * SCALE_270));
  }

  // SV0/SV2 両方完了したらフィードバック送信
  if (all_done) {
    if      (pos_state == POS_PICKUP)  pos_state = POS_PICKUP_DONE;
    else if (pos_state == POS_YAGURA)  pos_state = POS_YAGURA_DONE;
    else if (pos_state == POS_HONMARU) pos_state = POS_HONMARU_DONE;

    unsigned char dest = (pos_state == POS_PICKUP_DONE)  ? 0x00
                       : (pos_state == POS_YAGURA_DONE)  ? 0x01
                                                          : 0x02;
    sendFeedback(FB_BASE + 0x01, dest);
  }
}

// =====================================================================
// リングハンド更新（SV1）
// =====================================================================
void updateHandServo() {
  if (hand_state == HAND_STOPPED || hand_state == HAND_OPEN_DONE || hand_state == HAND_CLOSE_DONE) {
    return;
  }

  uint32_t elapsed = millis() - motionStartTimeHand;

  if (elapsed >= motionDurationHand) {
    currentAngles[2] = targetAngles[2];
    if (hand_state == HAND_OPEN) {
      hand_state = HAND_OPEN_DONE;
    } else if (hand_state == HAND_CLOSE) {
      hand_state   = HAND_CLOSE_DONE;
      gripFailSent = false;
    }
    sendFeedback(FB_BASE + 0x00, hand_state == HAND_OPEN_DONE ? 0x01 : 0x00);
  } else {
    float t    = (float)elapsed / (float)motionDurationHand;
    float ease = (1.0f - cosf(PI * t)) / 2.0f;
    currentAngles[2] = startAngleHand + (targetAngles[2] - startAngleHand) * ease;
  }

  servos[2]->write((int)roundf(currentAngles[2]));
}

// =====================================================================
// 櫓ハンド更新（SV3）
// =====================================================================
void updateYaguraHandServo() {
  if (sv3_state == HAND_STOPPED || sv3_state == HAND_OPEN_DONE || sv3_state == HAND_CLOSE_DONE) {
    return;
  }

  uint32_t elapsed = millis() - motionStartTimeSv3;

  if (elapsed >= motionDurationSv3) {
    currentSv3Angle = targetSv3Angle;
    if (sv3_state == HAND_OPEN)  sv3_state = HAND_OPEN_DONE;
    else if (sv3_state == HAND_CLOSE) sv3_state = HAND_CLOSE_DONE;
    sendFeedback(FB_BASE + 0x03, sv3_state == HAND_OPEN_DONE ? 0x01 : 0x00);
  } else {
    float t    = (float)elapsed / (float)motionDurationSv3;
    float ease = (1.0f - cosf(PI * t)) / 2.0f;
    currentSv3Angle = startSv3Angle + (targetSv3Angle - startSv3Angle) * ease;
  }

  servo3.write((int)roundf(currentSv3Angle * SCALE_270));
}
