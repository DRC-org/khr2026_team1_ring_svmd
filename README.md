リングハンドと櫓ハンドのサーボ制御コード

sv1がハンドの開閉
sv2が根元のサーボ
sv0が先のサーボ
sv3は櫓ハンド

{
  "servo_settings": {
    "pickup": {
      "sv0": 199,
      "sv2": 264
    },
    "yagura_picdown": {
      "sv0": 30,
      "sv2": 60
    },
    "honmaru": {
      "sv0": 30,
      "sv2": 74
    },
    "sv1_control": {
      "open": 120,
      "close": 180
    },
    "sv3_control": {
      "open": 0,
      "close": 90
    }
  }
}