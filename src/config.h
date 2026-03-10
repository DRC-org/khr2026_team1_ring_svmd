#pragma once

// =====================================================================
// ボードごとのサーボ角度設定
// =====================================================================

struct BoardConfig {
  float pos_pickup_sv0, pos_pickup_sv2;
  float pos_yagura_sv0, pos_yagura_sv2;
  float pos_honmaru_sv0, pos_honmaru_sv2;
  float yagura_open, yagura_close;
  float hand_open, hand_close, hand_grip;
};

// 後基板（CAN 0x401, ID=1）の絶対値
const BoardConfig CONFIG_401 = {
  197.0f, 270.0f,  // pickup  SV0, SV2
   40.0f,  50.0f,  // yagura  SV0, SV2
   25.0f,  80.0f,  // honmaru SV0, SV2
    0.0f,  85.0f,  // yagura hand open, close
  170.0f,  60.0f, 115.0f,  // hand open, close, grip
};

// 前基板（CAN 0x400, ID=0）のサーボごとオフセット（401の値に何度足すか）
struct ServoOffsets {
  float sv0, sv2, sv3, sv1;
};
const ServoOffsets OFFSETS_400 = { -3.0f, 4.0f, 20.0f, 0.0f };
