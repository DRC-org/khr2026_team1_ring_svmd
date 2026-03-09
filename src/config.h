#pragma once

// =====================================================================
// ボードごとのサーボ角度設定
// BOARD_ID = DIPスイッチで決まる数値（0, 1, ...）
// =====================================================================

struct BoardConfig {
  // 位置サーボ
  float pos_pickup_sv0;
  float pos_pickup_sv2;
  float pos_yagura_sv0;
  float pos_yagura_sv2;
  float pos_honmaru_sv0;
  float pos_honmaru_sv2;
  // リングハンド（SV1）
  float hand_open;
  float hand_close;
  float hand_grip;  // 把握成功確認角度
  // 櫓ハンド（SV3）
  float yagura_open;
  float yagura_close;
};

// インデックス = BOARD_ID（DIPスイッチ値）
const BoardConfig BOARD_CONFIGS[] = {
  // ---- ID=0: 前基板 ----
  {
    190.0f, 268.0f,  // pickup  SV0, SV2
     40.0f,  46.0f,  // yagura  SV0, SV2
     25.0f,  76.0f,  // honmaru SV0, SV2
    170.0f, 70.0f, 115.0f,  // hand open(g), close(r), grip
      0.0f,  95.0f,          // yagura hand open, close
  },
  // ---- ID=1: 後基板 ----
  {
    199.0f, 264.0f,  // pickup  SV0, SV2
     40.0f,  50.0f,  // yagura  SV0, SV2
     25.0f,  80.0f,  // honmaru SV0, SV2
    170.0f, 70.0f, 115.0f,  // hand open(g), close(r), grip
      0.0f,  80.0f,          // yagura hand open, close
  },
};
