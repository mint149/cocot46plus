/*
Copyright 2022 aki27

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include QMK_KEYBOARD_H
#include <stdio.h>
#include "quantum.h"

// レイヤー定義
#define _MAC 0
#define _WINDOWS 1
#define _LOWER 2
#define _RAISE 3
#define _ADJUST 4

#define LW_MHEN LT(1,KC_T)  // lower
#define RS_HENK LT(2,KC_T)  // raise
#define DEL_ALT ALT_T(KC_DEL)

/*
#define CPI_SW USER00
#define SCRL_SW USER01
#define ROT_R15 USER02
#define ROT_L15 USER03
#define SCRL_MO USER04
#define SCRL_TO USER05
#define SCRL_IN USER06
*/

// 長いキーをマクロに
#define SCRLTRG LT(_SCROLL,KC_BTN3)
#define NOSPACE _______
#define DELETED _______
#define NEXTTAB LCTL(KC_TAB)
#define PREVTAB LCTL(LSFT(KC_TAB))
#define NEXTXLS LCTL(KC_PGDN)
#define PREVXLS LCTL(KC_PGUP)

// キーコード定義
enum custom_keycodes {
  MAC = SAFE_RANGE,
  WINDOWS,
  IMEON,
  IMEOFF,
  TGL_JIS
};

bool ime_off_only = false;
bool ime_on_only = false;
bool jis_mode = false;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_MAC] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
       KC_TAB,    KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,                                          KC_Y,    KC_U,    KC_I,    KC_O,   KC_P,  KC_BSPC,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_LCTL,    KC_A,    KC_S,    KC_D,    KC_F,    KC_G,                                          KC_H,    KC_J,    KC_K,    KC_L, KC_SCLN,  KC_ENT,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_LSFT,    KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,                                          KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, KC_QUOT,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        DELETED, KC_LGUI,  IMEOFF,  KC_SPC, KC_MS_BTN1,             KC_MS_BTN2,   KC_RALT,   IMEON, DELETED, DELETED,
                                                                 KC_WH_D, KC_MS_BTN3,  KC_WH_U, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    ),
  [_WINDOWS] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
       KC_TAB,    KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,                                          KC_Y,    KC_U,    KC_I,    KC_O,   KC_P,  KC_BSPC,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_LCTL,    KC_A,    KC_S,    KC_D,    KC_F,    KC_G,                                          KC_H,    KC_J,    KC_K,    KC_L, KC_SCLN,  KC_ENT,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      KC_LSFT,    KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,                                          KC_N,    KC_M, KC_COMM,  KC_DOT, KC_SLSH, KC_QUOT,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        DELETED, KC_LALT,  IMEOFF,  KC_SPC, KC_MS_BTN1,             KC_MS_BTN2,   KC_RGUI,   IMEON, DELETED, DELETED,
                                                                 KC_WH_U, KC_MS_BTN3,  KC_WH_D, XXXXXXX, XXXXXXX, XXXXXXX
                                                            //`--------------'  `--------------'
    ),
  [_LOWER] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
       KC_ESC, _______,   KC_F2,   KC_F3,   KC_F4,   KC_F5,                                          KC_1,    KC_2,    KC_3,    KC_4,    KC_5,  KC_DEL, 
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,   KC_F6,   KC_F7,   KC_F8,   KC_F9,  KC_F10,                                          KC_6,    KC_7,    KC_8,    KC_9,    KC_0, _______, 
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______,  KC_F11,  KC_F12, _______, _______, _______,                                       KC_MINS, KC_EQL , KC_LBRC, KC_RBRC, KC_BSLS,  KC_GRV,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, _______,  _______,   _______,                 _______,  _______, _______, _______, _______,
                                                                 _______, _______,  _______, _______, _______, _______
                                                            //`--------------'  `--------------'
    ),
  [_RAISE] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______, _______, _______, _______, _______, _______,                                       KC_HOME, KC_PGDN, KC_PGUP,  KC_END, _______, KC_DEL , 
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______, _______,  CPI_SW, SCRL_SW, _______, _______,                                       KC_LEFT, KC_DOWN, KC_UP  ,KC_RIGHT, _______, _______, 
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      _______, _______, _______, _______, _______, _______,                                       KC_MINS, KC_EQL , KC_LBRC, KC_RBRC, KC_BSLS, KC_GRV ,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, _______,  _______,   _______,             _______,  _______, _______, _______,  _______,
                                                                 _______, _______,  _______, _______, _______, _______
                                                            //`--------------'  `--------------'
    ),
  [_ADJUST] = LAYOUT(
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, WINDOWS, XXXXXXX, XXXXXXX, RGB_TOG,                                       SCRL_TO,  CPI_SW, SCRL_SW, ROT_L15, ROT_R15, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, RGB_VAI, RGB_SAI, RGB_HUI, RGB_MOD,                                       SCRL_MO, TGL_JIS, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
      XXXXXXX, XXXXXXX, RGB_VAD, RGB_SAD, RGB_HUD,RGB_RMOD,                                       SCRL_IN,     MAC, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
  //|-------------------------------------------------------|                                   |-------------------------------------------------------|
                        _______, _______, _______,  _______,   _______,             _______,  _______, _______, _______,  _______,
                                                                 _______, _______,  _______, _______, _______, _______
                                                            //`--------------'  `--------------'
    )
};

keyevent_t encoder1_ccw = {
    .key = (keypos_t){.row = 4, .col = 2},
    .pressed = false
};

keyevent_t encoder1_cw = {
    .key = (keypos_t){.row = 4, .col = 5},
    .pressed = false
};

bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 0) { /* First encoder */
        if (clockwise) {
            encoder1_cw.pressed = true;
            encoder1_cw.time = (timer_read() | 1);
            action_exec(encoder1_cw);
        } else {
            encoder1_ccw.pressed = true;
            encoder1_ccw.time = (timer_read() | 1);
            action_exec(encoder1_ccw);
        }
    }

    return true;
}


void matrix_scan_user(void) {

    if (IS_PRESSED(encoder1_ccw)) {
        encoder1_ccw.pressed = false;
        encoder1_ccw.time = (timer_read() | 1);
        action_exec(encoder1_ccw);
    }

    if (IS_PRESSED(encoder1_cw)) {
        encoder1_cw.pressed = false;
        encoder1_cw.time = (timer_read() | 1);
        action_exec(encoder1_cw);
    }

}



layer_state_t layer_state_set_user(layer_state_t state) {
    switch (get_highest_layer(state)) {
    case _MAC:
        rgblight_sethsv_range(HSV_BLUE, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    case _WINDOWS:
        rgblight_sethsv_range(HSV_RED, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    case _LOWER:
        rgblight_sethsv_range(HSV_YELLOW, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    case _RAISE:
        rgblight_sethsv_range(HSV_GREEN, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    case _ADJUST:
        rgblight_sethsv_range(HSV_CYAN, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    // case _Layer6:
    //     rgblight_sethsv_range(HSV_ORANGE, 0, 2);
    //     cocot_set_scroll_mode(false);
    //     break;
    default:
        rgblight_sethsv_range( 0, 0, 0, 0, 2);
        cocot_set_scroll_mode(false);
        break;
    }
    rgblight_set_effect_range( 2, 10);
      return state;
};


#ifdef OLED_ENABLE
bool oled_task_user(void) {
    // oled_write_layer_state();
    render_logo();

    oled_write_P(PSTR("Layer:"), false);

    switch (get_highest_layer(layer_state | default_layer_state)) {
        case _MAC:
            oled_write_P(PSTR("Mac   "), false);
            break;
        case _WINDOWS:
            oled_write_P(PSTR("Win   "), false);
            break;
        case _LOWER:
            oled_write_P(PSTR("Lower "), false);
            break;
        case _RAISE:
            oled_write_P(PSTR("Raise "), false);
            break;
        case _ADJUST:
            oled_write_P(PSTR("Adjust"), false);
            break;
        default:
            oled_write_P(PSTR("Undef "), false);
            break;
    }

    oled_write_P(PSTR(" Mode:"), false);

    if(jis_mode){
      oled_write_P(PSTR("JIS"), false);
    }else{
      oled_write_P(PSTR(" US"), false);
    }

    return false;
}
#endif

/* Copyright 2018-2020 eswai <@eswai>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  OSで日本語キーボード(logical bit pairing)と設定/認識されているキーボードで、
  USキーキャップの文字、記号(typewriter pairing)を正しく出力する。
  例: Shift + 2 で @ を入力する
  変換された文字はキーリピートが無効です。
*/

// #include QMK_KEYBOARD_H
#include "keymap_japanese.h"

const uint16_t us2jis[][2] = {
  {KC_LPRN, JP_LPRN},
  {KC_RPRN, JP_RPRN},
  {KC_AT,   JP_AT},
  {KC_LBRC, JP_LBRC},
  {KC_RBRC, JP_RBRC},
  {KC_LCBR, JP_LCBR},
  {KC_RCBR, JP_RCBR},
  {KC_MINS, JP_MINS},
  {KC_EQL,  JP_EQL},
  {KC_BSLS, JP_BSLS},
  {KC_SCLN, JP_SCLN},
  {KC_QUOT, JP_QUOT},
  {KC_GRV,  JP_GRV},
  {KC_PLUS, JP_PLUS},
  {KC_COLN, JP_COLN},
  {KC_UNDS, JP_UNDS},
  {KC_PIPE, JP_PIPE},
  {KC_DQT,  JP_DQUO},
  {KC_ASTR, JP_ASTR},
  {KC_TILD, JP_TILD},
  {KC_AMPR, JP_AMPR},
  {KC_CIRC, JP_CIRC},
};

bool twpair_on_jis(uint16_t keycode, keyrecord_t *record) {
  if (!record->event.pressed) return true;

  uint16_t skeycode; // シフトビットを反映したキーコード
  bool lshifted = keyboard_report->mods & MOD_BIT(KC_LSFT); // シフトキーの状態
  bool rshifted = keyboard_report->mods & MOD_BIT(KC_RSFT);
  bool shifted = lshifted | rshifted;

  if (shifted) {
    skeycode = QK_LSFT | keycode;
  } else {
    skeycode = keycode;
  }

  for (int i = 0; i < sizeof(us2jis) / sizeof(us2jis[0]); i++) {
    if (us2jis[i][0] == skeycode) {
      unregister_code(KC_LSFT);
      unregister_code(KC_RSFT);
      if ((us2jis[i][1] & QK_LSFT) == QK_LSFT || (us2jis[i][1] & QK_RSFT) == QK_RSFT) {
        register_code(KC_LSFT);
        tap_code(us2jis[i][1]);
        unregister_code(KC_LSFT);
      } else {
        tap_code(us2jis[i][1]);
      }
      if (lshifted) register_code(KC_LSFT);
      if (rshifted) register_code(KC_RSFT);
      return false;
    }
  }

  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  bool result = false;
  // IMEキーの単押し判定用Switch
  switch (keycode) {
    case IMEOFF:
      ime_on_only = false;
      break;
    case IMEON:
      ime_off_only = false;
      break;
    default:
      ime_off_only = false;
      ime_on_only = false;
      break;
  }
  switch (keycode) {
    case MAC:
      if (record->event.pressed) {
        default_layer_set(1UL<<_MAC);
      }
      return false;
      break;
    case WINDOWS:
      if (record->event.pressed) {
        default_layer_set(1UL<<_WINDOWS);
      }
      return false;
      break;
    case IMEOFF:
      if (record->event.pressed) {
        ime_off_only = true;
        layer_on(_LOWER);
        update_tri_layer(_LOWER, _RAISE, _ADJUST);
      } else {
        layer_off(_LOWER);
        update_tri_layer(_LOWER, _RAISE, _ADJUST);
    
        if (ime_off_only) {
          switch (get_highest_layer(default_layer_state)) {
            case _WINDOWS:
              tap_code(KC_INT5);
              break;
            case _MAC: 
              tap_code(KC_LNG2);
              break;
            default:
              break;
          }
        }
        ime_off_only = false;
      }
      return false;
      break;
    case IMEON:
      if (record->event.pressed) {
        ime_on_only = true;
        layer_on(_RAISE);
        update_tri_layer(_LOWER, _RAISE, _ADJUST);
      } else {
        layer_off(_RAISE);
        update_tri_layer(_LOWER, _RAISE, _ADJUST);
    
        if (ime_on_only) {
          switch (get_highest_layer(default_layer_state)) {
            case _WINDOWS:
              tap_code(KC_INT4);
              break;
            case _MAC: 
              tap_code(KC_LNG1);
              break;
            default:
              break;
          }
        }
        ime_on_only = false;
      }
      return false;
      break;
    case TGL_JIS:
      if (record->event.pressed) {
        jis_mode = !jis_mode;
      }

      return false;
      break;
    default:
      if(jis_mode){
        result = twpair_on_jis(keycode, record);
      }else{
        result = true;
      }
      break;
  }

  return result;
}
