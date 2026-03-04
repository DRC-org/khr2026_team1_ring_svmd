#include <Arduino.h>
#include <Arduino_CAN.h>
#include <FastLED_NeoPixel.h>
#include <Servo.h>

// void readSwitch();
void updatePosServos();
void updateHandServo();

// サーボ関係
#define SV0 3   // 関節 1 (150 kg)
#define SV1 6   // 関節 2 (20 kg)
#define SV2 10  // ハンド
#define SV3 9
// LED 関係
#define LED_PIN 13
#define RGB 8
// 通信関係
#define CAN_RX 5
#define CAN_TX 4
// 入力関係
#define SW0 8
#define SW1 9
#define SW2 1
#define SW3 0

Servo servo0;
Servo servo1;
Servo servo2;

FastLED_NeoPixel<1, RGB, NEO_GRB> strip;  // RGBLED 制御用

unsigned int CAN_ID = 0x401;  // CANのID（サーボモータドライバは 0x400 番台）

const float SERVO_MAX_SPEED = 5.0f;  // 最大角速度（度/step）
const float SERVO_ACCEL = 0.4f;      // 加速度（度/step²）
const float SERVO_LINEAR_SPEED = 5.0f;

Servo* servos[3] = {&servo0, &servo1, &servo2};
float currentAngles[3] = {80.0f, 80.0f, 80.0f};
float targetAngles[3] = {80.0f, 80.0f, 80.0f};
float servoVelocities[3] = {0.0f, 0.0f, 0.0f};

// 0: PICKUP, 1: YAGURA, 2: HONMARU, 3: STOPPED, 4: PICKUP_DONE, 5: YAGURA_DONE,
// 6: HONMARU_DONE
int8_t pos_state = 0;

// 0: OPEN, 1: CLOSE, 2: STOPPED, 3: OPEN_DONE, 4: CLOSE_DONE
int8_t hand_state = 0;

unsigned long lastUpdateMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SV0, OUTPUT);
  pinMode(SV1, OUTPUT);
  pinMode(SV2, OUTPUT);
  // pinMode(SV3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RGB, OUTPUT);
  pinMode(SW0, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(100);

  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();

  servo0.attach(SV0);
  servo1.attach(SV1);
  servo2.attach(SV2);
  Serial.println("Setup done.");

  servo0.write(12);
  servo1.write(0);
  servo2.write(0);

  Serial.println("Waiting for power...");

  delay(50);

  // TODO: ホーミング

  lastUpdateMs = millis();

  if (!CAN.begin(CanBitRate::BR_1000k)) {
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

  // readSwitch();  // DIP スイッチの読み出して CAN_ID を指定
}

void loop() {
  if (CAN.available()) {
    CanMsg const msg = CAN.read();  // 受信した CAN の読み出し
    unsigned int rxId = msg.id;

    if (rxId == CAN_ID) {
      unsigned char command = msg.data[0];
      unsigned char param = msg.data[1];

      if (command == 0x00) {  // ハンド 1 を移動
        if (param == 0x00) {  // ピックアップ
          targetAngles[0] = 12.0f;
          targetAngles[1] = 0.0f;
          pos_state = 0;
        } else if (param == 0x01) {  // やぐら
          targetAngles[0] = 12.0f;
          targetAngles[1] = 0.0f;
          pos_state = 1;
        } else if (param == 0x02) {  // 本丸
          targetAngles[0] = 12.0f;
          targetAngles[1] = 0.0f;
          pos_state = 2;
        }
      } else if (command == 0x01) {  // ハンド 1 を停止
        targetAngles[0] = currentAngles[0];
        targetAngles[1] = currentAngles[1];
        pos_state = 3;
      } else if (command == 0x10) {  // ハンド 1 を閉じる
        targetAngles[2] = 80.0f;
        hand_state = 1;
      } else if (command == 0x11) {  // ハンド 1 を開く
        targetAngles[2] = 0.0f;
        hand_state = 0;
      } else if (command == 0x12) {  // ハンド 1 を停止
        targetAngles[2] = currentAngles[2];
        servoVelocities[2] = 0.0f;
        hand_state = 2;
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

    updatePosServos();
    updateHandServo();
  }
}

// DIPスイッチを読み取ってCAN_IDを指定する
// void readSwitch() {
//   unsigned int readNumber = !digitalRead(SW0) + 2 * !digitalRead(SW1) +
//                             4 * !digitalRead(SW2) + 8 * !digitalRead(SW3);
//   switch (readNumber) {
//     case 0:
//       strip.setPixelColor(0, strip.Color(255, 100, 0));
//       break;
//     case 1:
//       strip.setPixelColor(0, strip.Color(255, 255, 0));
//       break;
//     case 2:
//       strip.setPixelColor(0, strip.Color(0, 255, 0));
//       break;
//     case 3:
//       strip.setPixelColor(0, strip.Color(0, 0, 255));
//       break;
//   }
//   strip.show();
//   CAN_ID = 0x300 + readNumber;
//   Serial.print("CAN_ID: ");
//   Serial.println(CAN_ID);
// }

void updatePosServos() {
  for (int i = 0; i < 2; i++) {
    if (pos_state == 3 || pos_state == 4 || pos_state == 5 || pos_state == 6) {
      targetAngles[i] = currentAngles[i];
      continue;
    }

    float diff = targetAngles[i] - currentAngles[i];
    float dist = fabsf(diff);
    float vel = servoVelocities[i];

    if (dist < 0.5f) {
      currentAngles[i] = targetAngles[i];
      servoVelocities[i] = 0.0f;

      if (i == 0) {
        if (pos_state == 0) {
          pos_state = 4;  // PICKUP_DONE
        } else if (pos_state == 1) {
          pos_state = 5;  // YAGURA_DONE
        } else if (pos_state == 2) {
          pos_state = 6;  // HONMARU_DONE
        }

        CanMsg msg;
        msg.id = 0x000;
        msg.data_length = 8;

        msg.data[0] = 0x4A;
        msg.data[1] = pos_state == 4 ? 0x00 : (pos_state == 5 ? 0x01 : 0x02);

        CAN.write(msg);
      }
    } else {
      float dir = (diff > 0.0f) ? 1.0f : -1.0f;

      if (vel * diff >= 0.0f) {
        // 目標方向に進んでいる: 制動距離 v²/(2a) を基に加速か減速かを判断
        float stopDist = (vel * vel) / (2.0f * SERVO_ACCEL);
        if (dist <= stopDist + SERVO_ACCEL) {
          // 減速フェーズ (Ease-out)
          vel -= dir * SERVO_ACCEL;
          if ((dir > 0.0f && vel < 0.0f) || (dir < 0.0f && vel > 0.0f))
            vel = 0.0f;
        } else {
          // 加速フェーズ (Ease-in) または定速
          vel = constrain(vel + dir * SERVO_ACCEL, -SERVO_MAX_SPEED,
                          SERVO_MAX_SPEED);
        }
      } else {
        // 逆方向に動いている: 目標方向へ加速
        vel = constrain(vel + dir * SERVO_ACCEL, -SERVO_MAX_SPEED,
                        SERVO_MAX_SPEED);
      }

      // オーバーシュート防止
      if (fabsf(vel) >= dist) {
        currentAngles[i] = targetAngles[i];
        vel = 0.0f;
      } else {
        currentAngles[i] += vel;
      }
      servoVelocities[i] = vel;
    }

    servos[i]->write((int)roundf(currentAngles[i]));
  }
}

void updateHandServo() {
  if (hand_state == 2 || hand_state == 3 || hand_state == 4) {
    return;
  }

  float diff = targetAngles[2] - currentAngles[2];
  if (fabsf(diff) < 0.5f) {
    currentAngles[2] = targetAngles[2];
    if (hand_state == 0) {
      hand_state = 3;  // OPEN_DONE
    } else if (hand_state == 1) {
      hand_state = 4;  // CLOSE_DONE
    }

    CanMsg msg;
    msg.id = 0x000;
    msg.data_length = 8;

    msg.data[0] = 0x4B;
    msg.data[1] = hand_state == 3 ? 0x01 : 0x00;

    CAN.write(msg);
  } else {
    float step = constrain(diff, -SERVO_LINEAR_SPEED, SERVO_LINEAR_SPEED);
    currentAngles[2] += step;
  }
  servos[2]->write((int)roundf(currentAngles[2]));
}
