#include <Arduino.h>
#include <FastLED_NeoPixel.h>
#include <SPI.h>
#include <Servo.h>
#include <mcp_can.h>

void updatePosServos();
void updateHandServo();
void updateYaguraHandServo();
void startPosMotion();

// #define PGOOD 2
#define INT 3
#define SV0 4
#define SV1 5
#define SV2 6
#define SV3 7
#define SV4 8
#define RGB 9
#define CS 10
// #define MOSI 11
// #define MISO 12
// #define SCLK 13

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

FastLED_NeoPixel<1, RGB, NEO_GRB> strip;  // RGBLED 制御用

MCP_CAN CAN(CS);

// 書き込む基板に合わせて変更する（前: 0 / 後: 1）
#define BOARD_CAN_ID 1

unsigned int CAN_ID = 0x400 + BOARD_CAN_ID;
unsigned char FB_BASE = 0x40 + BOARD_CAN_ID * 0x0A;

// サイン波イージングの平均角速度
const float SERVO_AVG_SPEED = 45.0f;     // 度/秒
const float SERVO_AVG_SPEED_SV3 = 20.0f;  // sv3用（ゆっくり）
const uint32_t UPDATE_INTERVAL_MS = 10;  // 更新間隔 (100Hz)

Servo* servos[3] = {&servo0, &servo1, &servo2};
float currentAngles[3] = {80.0f, 80.0f, 80.0f};
float targetAngles[3] = {80.0f, 80.0f, 80.0f};

float startAngles[2] = {80.0f, 80.0f};
uint32_t motionStartTime[2] = {0, 0};
uint32_t motionDuration[2] = {0, 0};

float startAngleHand = 80.0f;
uint32_t motionStartTimeHand = 0;
uint32_t motionDurationHand = 0;

// 0: PICKUP, 1: YAGURA, 2: HONMARU, 3: STOPPED, 4: PICKUP_DONE, 5: YAGURA_DONE,
// 6: HONMARU_DONE
int8_t pos_state = 0;

// 0: OPEN, 1: CLOSE, 2: STOPPED, 3: OPEN_DONE, 4: CLOSE_DONE
int8_t hand_state = 0;

float currentSv3Angle = 0.0f;
float targetSv3Angle = 0.0f;
float startSv3Angle = 0.0f;
uint32_t motionStartTimeSv3 = 0;
uint32_t motionDurationSv3 = 0;
// 0: OPEN, 1: CLOSE, 2: STOPPED, 3: OPEN_DONE, 4: CLOSE_DONE
int8_t sv3_state = 0;

uint32_t lastUpdateTime = 0;
bool gripFailSent = false;  // 把持失敗通知済みフラグ

// pos サーボ (0,1) のサイン波モーションを開始する
void startPosMotion() {
  uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    startAngles[i] = currentAngles[i];
    float dist = fabsf(targetAngles[i] - currentAngles[i]);
    motionDuration[i] = (uint32_t)(dist / SERVO_AVG_SPEED * 1000.0f);
    motionStartTime[i] = now;
  }
}

void setup() {
  Serial.begin(115200);

  // pinMode(PGOOD, INPUT);
  pinMode(INT, INPUT);
  pinMode(SV0, OUTPUT);
  pinMode(SV1, OUTPUT);
  pinMode(SV2, OUTPUT);
  pinMode(SV3, OUTPUT);
  pinMode(SV4, INPUT_PULLUP);
  pinMode(RGB, OUTPUT);
  pinMode(CS, OUTPUT);
  // pinMode(MOSI, OUTPUT);
  // pinMode(MISO, INPUT);
  // pinMode(SCLK, OUTPUT);

  strip.begin();
  strip.setBrightness(100);

  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();

  servo0.attach(SV0);
  servo1.attach(SV1);
  servo2.attach(SV2);
  servo3.attach(SV3);

  servo0.write(0);
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CAN.begin(...) failed.");
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

void loop() {
  if (!digitalRead(INT)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    CAN.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == CAN_ID) {
      unsigned char command = rxBuf[0];
      unsigned char param = rxBuf[1];

      if (command == 0x00) {  // ハンド 1 を移動
        if (param == 0x00) {  // ピックアップ
          targetAngles[0] = 199.0f;
          targetAngles[1] = 264.0f;
          pos_state = 0;
        } else if (param == 0x01) {  // やぐら
          targetAngles[0] = 30.0f;
          targetAngles[1] = 60.0f;
          pos_state = 1;
        } else if (param == 0x02) {  // 本丸
          targetAngles[0] = 30.0f;
          targetAngles[1] = 74.0f;
          pos_state = 2;
        }
        startPosMotion();
      } else if (command == 0x01) {  // ハンド 1 を停止
        targetAngles[0] = currentAngles[0];
        targetAngles[1] = currentAngles[1];
        pos_state = 3;
      } else if (command == 0x10) {  // ハンド 1 を閉じる
        targetAngles[2] = 180.0f;
        hand_state = 1;
        startAngleHand = currentAngles[2];
        motionDurationHand = (uint32_t)(fabsf(targetAngles[2] - currentAngles[2]) / SERVO_AVG_SPEED * 1000.0f);
        motionStartTimeHand = millis();
      } else if (command == 0x11) {  // ハンド 1 を開く
        targetAngles[2] = 120.0f;
        hand_state = 0;
        startAngleHand = currentAngles[2];
        motionDurationHand = (uint32_t)(fabsf(targetAngles[2] - currentAngles[2]) / SERVO_AVG_SPEED * 1000.0f);
        motionStartTimeHand = millis();
      } else if (command == 0x12) {  // ハンド 1 を停止
        targetAngles[2] = currentAngles[2];
        hand_state = 2;
      } else if (command == 0x20) {  // 櫓ハンドを閉じる
        targetSv3Angle = 90.0f;
        sv3_state = 1;
        startSv3Angle = currentSv3Angle;
        motionDurationSv3 = (uint32_t)(fabsf(targetSv3Angle - currentSv3Angle) / SERVO_AVG_SPEED_SV3 * 1000.0f);
        motionStartTimeSv3 = millis();
      } else if (command == 0x21) {  // 櫓ハンドを開く
        targetSv3Angle = 0.0f;
        sv3_state = 0;
        startSv3Angle = currentSv3Angle;
        motionDurationSv3 = (uint32_t)(fabsf(targetSv3Angle - currentSv3Angle) / SERVO_AVG_SPEED_SV3 * 1000.0f);
        motionStartTimeSv3 = millis();
      } else if (command == 0x22) {  // 櫓ハンドを停止
        targetSv3Angle = currentSv3Angle;
        sv3_state = 2;
      }

      // if (command == 0x10) {  // ハンド 2 を閉じる
      //   targetAngles[1] = 80.0f;
      //   states[1] = 1;               // CLOSE
      // } else if (command == 0x11) {  // ハンド 2 を開く
      //   targetAngles[1] = 0.0f;
      //   states[1] = 0;               // OPEN
      // } else if (command == 0x12) {  // ハンド 2 を停止
      //   targetAngles[1] = currentAngles[1];
      //   servoVelocities[1] = 0.0f;
      //   states[1] = 2;  // STOPPED
      // }
    }
  }

  uint32_t now = millis();
  if (now - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    lastUpdateTime = now;
    updatePosServos();
    updateHandServo();
    updateYaguraHandServo();

    if (hand_state == 4 && !gripFailSent && !digitalRead(SV4)) {
      unsigned char txBuf[8] = {0x00};
      txBuf[0] = FB_BASE + 0x02;  // 把持失敗
      CAN.sendMsgBuf(0x000, 0, 8, txBuf);
      gripFailSent = true;
    }
  }
}


void updatePosServos() {
  uint32_t now = millis();

  for (int i = 0; i < 2; i++) {
    if (pos_state == 3 || pos_state == 4 || pos_state == 5 || pos_state == 6) {
      targetAngles[i] = currentAngles[i];
      continue;
    }

    uint32_t elapsed = now - motionStartTime[i];

    if (elapsed >= motionDuration[i]) {
      currentAngles[i] = targetAngles[i];

      if (i == 0) {
        if (pos_state == 0) {
          pos_state = 4;  // PICKUP_DONE
        } else if (pos_state == 1) {
          pos_state = 5;  // YAGURA_DONE
        } else if (pos_state == 2) {
          pos_state = 6;  // HONMARU_DONE
        }

        unsigned char txBuf[8] = {0x00};

        txBuf[0] = FB_BASE + 0x01;  // 位置完了
        txBuf[1] = pos_state == 4 ? 0x00 : (pos_state == 5 ? 0x01 : 0x02);

        CAN.sendMsgBuf(0x000, 0, 8, txBuf);
      }
    } else {
      float t = (float)elapsed / (float)motionDuration[i];
      // サイン波イージング: 開始・終了を滑らかにして慣性を抑制
      float ease = (1.0f - cosf(PI * t)) / 2.0f;
      currentAngles[i] =
          startAngles[i] + (targetAngles[i] - startAngles[i]) * ease;
    }

    servos[i]->write((int)roundf(currentAngles[i]));
  }
}

void updateYaguraHandServo() {
  if (sv3_state == 2 || sv3_state == 3 || sv3_state == 4) {
    return;
  }

  uint32_t elapsed = millis() - motionStartTimeSv3;

  if (elapsed >= motionDurationSv3) {
    currentSv3Angle = targetSv3Angle;
    if (sv3_state == 0) {
      sv3_state = 3;  // OPEN_DONE
    } else if (sv3_state == 1) {
      sv3_state = 4;  // CLOSE_DONE
    }

    unsigned char txBuf[8] = {0x00};

    txBuf[0] = FB_BASE + 0x03;  // 櫓ハンド完了
    txBuf[1] = sv3_state == 3 ? 0x01 : 0x00;

    CAN.sendMsgBuf(0x000, 0, 8, txBuf);
  } else {
    float t = (float)elapsed / (float)motionDurationSv3;
    float ease = (1.0f - cosf(PI * t)) / 2.0f;
    currentSv3Angle = startSv3Angle + (targetSv3Angle - startSv3Angle) * ease;
  }
  servo3.write((int)roundf(currentSv3Angle));
}

void updateHandServo() {
  if (hand_state == 2 || hand_state == 3 || hand_state == 4) {
    return;
  }

  uint32_t elapsed = millis() - motionStartTimeHand;

  if (elapsed >= motionDurationHand) {
    currentAngles[2] = targetAngles[2];
    if (hand_state == 0) {
      hand_state = 3;  // OPEN_DONE
    } else if (hand_state == 1) {
      hand_state = 4;  // CLOSE_DONE
      gripFailSent = false;
    }

    unsigned char txBuf[8] = {0x00};

    txBuf[0] = FB_BASE + 0x00;  // 開閉完了
    txBuf[1] = hand_state == 3 ? 0x01 : 0x00;

    CAN.sendMsgBuf(0x000, 0, 8, txBuf);
  } else {
    float t = (float)elapsed / (float)motionDurationHand;
    float ease = (1.0f - cosf(PI * t)) / 2.0f;
    currentAngles[2] = startAngleHand + (targetAngles[2] - startAngleHand) * ease;
  }
  servos[2]->write((int)roundf(currentAngles[2]));
}
