#include <Arduino.h>
#include <FastLED_NeoPixel.h>
#include <SPI.h>
#include <Servo.h>
#include <mcp_can.h>
#include "config.h"

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
// DIP スイッチ（A0〜A3）で CAN_ID を自動設定
// =====================================================================
#define SW0 A0
#define SW1 A1
#define SW2 A2
#define SW3 A3

unsigned int CAN_ID;
unsigned char FB_BASE;
const BoardConfig* cfg;

static BoardConfig s_config400;

void readSwitch() {
  unsigned int id = !digitalRead(SW0) + 2 * !digitalRead(SW1) +
                    4 * !digitalRead(SW2) + 8 * !digitalRead(SW3);
  CAN_ID  = 0x400 + id;
  FB_BASE = 0x40 + id * 0x0A;
  if (id == 0) {
    s_config400 = {
      CONFIG_401.pos_pickup_sv0  + OFFSETS_400.sv0,
      CONFIG_401.pos_pickup_sv2  + OFFSETS_400.sv2,
      CONFIG_401.pos_yagura_sv0  + OFFSETS_400.sv0,
      CONFIG_401.pos_yagura_sv2  + OFFSETS_400.sv2,
      CONFIG_401.pos_honmaru_sv0 + OFFSETS_400.sv0,
      CONFIG_401.pos_honmaru_sv2 + OFFSETS_400.sv2,
      CONFIG_401.yagura_open     + OFFSETS_400.sv3,
      CONFIG_401.yagura_close    + OFFSETS_400.sv3,
      CONFIG_401.hand_open       + OFFSETS_400.sv1,
      CONFIG_401.hand_close      + OFFSETS_400.sv1,
      CONFIG_401.hand_grip       + OFFSETS_400.sv1,
    };
    cfg = &s_config400;
  } else {
    cfg = &CONFIG_401;
  }
}

// フィードバック識別子
// FB_BASE+0x00: 開閉完了
// FB_BASE+0x01: 位置完了
// FB_BASE+0x02: 把持失敗
// FB_BASE+0x03: 櫓ハンド完了

// サーボ角度は config.h の BoardConfig を参照 (cfg->pos_pickup_sv0 等)

// =====================================================================
// モーション定数
// =====================================================================
const float    SCALE_270         = 180.0f / 270.0f;  // 270°サーボ → servo.write() 変換係数
const float    POS_SERVO_SPEED   = 45.0f;   // 位置サーボ 平均角速度 [deg/s]
const float    YAGURA_SERVO_SPEED = 80.0f;  // 櫓ハンド 平均角速度 [deg/s]
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
bool     gripCheckPending   = false;
uint32_t gripCheckStartTime = 0;

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

  pinMode(SW0, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);

  readSwitch();

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

  // SV0を起動時の向きに合わせて最後に10度へ（オフセット込み）
  float sv0_offset = (CAN_ID == 0x400) ? OFFSETS_400.sv0 : 0.0f;
  servo0.write((int)roundf((10.0f + sv0_offset) * SCALE_270));
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
          targetAngles[0] = cfg->pos_pickup_sv0;
          targetAngles[1] = cfg->pos_pickup_sv2;
          pos_state = POS_PICKUP;
        } else if (param == 0x01) {
          targetAngles[0] = cfg->pos_yagura_sv0;
          targetAngles[1] = cfg->pos_yagura_sv2;
          pos_state = POS_YAGURA;
        } else if (param == 0x02) {
          targetAngles[0] = cfg->pos_honmaru_sv0;
          targetAngles[1] = cfg->pos_honmaru_sv2;
          pos_state = POS_HONMARU;
        }
        startPosMotion();

      } else if (command == 0x01) {  // 位置停止
        targetAngles[0] = currentAngles[0];
        targetAngles[1] = currentAngles[1];
        pos_state = POS_STOPPED;

      } else if (command == 0x10) {  // リングハンド 閉じる（即時）
        currentAngles[2] = cfg->hand_close;
        targetAngles[2]  = cfg->hand_close;
        servos[2]->write((int)roundf(cfg->hand_close));
        hand_state        = HAND_CLOSE_DONE;
        gripCheckPending  = false;
        sendFeedback(FB_BASE + 0x00, 0x00);

      } else if (command == 0x11) {  // リングハンド 開く → 1秒後に把握判定
        currentAngles[2]  = cfg->hand_open;
        targetAngles[2]   = cfg->hand_open;
        servos[2]->write((int)roundf(cfg->hand_open));
        hand_state        = HAND_OPEN_DONE;
        gripCheckPending  = true;
        gripCheckStartTime = millis();

      } else if (command == 0x12) {  // リングハンド 停止
        targetAngles[2] = currentAngles[2];
        hand_state = HAND_STOPPED;

      } else if (command == 0x20) {  // 櫓ハンド 閉じる
        targetSv3Angle     = cfg->yagura_close;
        sv3_state          = HAND_CLOSE;
        startSv3Angle      = currentSv3Angle;
        motionDurationSv3  = (uint32_t)(fabsf(cfg->yagura_close - currentSv3Angle) / YAGURA_SERVO_SPEED * 1000.0f);
        motionStartTimeSv3 = millis();

      } else if (command == 0x21) {  // 櫓ハンド 開く
        targetSv3Angle     = cfg->yagura_open;
        sv3_state          = HAND_OPEN;
        startSv3Angle      = currentSv3Angle;
        motionDurationSv3  = (uint32_t)(fabsf(cfg->yagura_open - currentSv3Angle) / YAGURA_SERVO_SPEED * 1000.0f);
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

    // open後1秒でSV4チェック: 非導通=把握成功, 導通=把握失敗
    if (gripCheckPending && now - gripCheckStartTime >= 1000) {
      gripCheckPending = false;
      if (!digitalRead(SV4)) {  // 導通 → 把握失敗 → 閉じる
        currentAngles[2] = cfg->hand_close;
        targetAngles[2]  = cfg->hand_close;
        servos[2]->write((int)roundf(cfg->hand_close));
        hand_state = HAND_CLOSE_DONE;
        sendFeedback(FB_BASE + 0x00, 0x00);  // 閉じ完了を通知
        sendFeedback(FB_BASE + 0x02, 0x00);  // 把握失敗を通知
      } else {                   // 非導通 → 把握成功
        currentAngles[2] = cfg->hand_grip;
        targetAngles[2]  = cfg->hand_grip;
        servos[2]->write((int)roundf(cfg->hand_grip));
        sendFeedback(FB_BASE + 0x00, 0x01);
      }
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
