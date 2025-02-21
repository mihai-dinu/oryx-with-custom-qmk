#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  ST_MACRO_3,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,        
    KC_TRANSPARENT, MT(MOD_LALT, KC_A),MT(MOD_LCTL, KC_S),MT(MOD_LSFT, KC_D),MT(MOD_LGUI, KC_F),KC_G,                                           KC_H,           MT(MOD_RGUI, KC_J),MT(MOD_RSFT, KC_K),MT(MOD_RCTL, KC_L),MT(MOD_LALT, KC_QUOTE),KC_SCLN,        
    CW_TOGG,        KC_Z,           TD(DANCE_0),    TD(DANCE_1),    TD(DANCE_2),    KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       TD(DANCE_3),    
                                                    KC_ENTER,       LT(2,KC_BSPC),                                  OSL(1),         KC_SPACE
  ),
  [1] = LAYOUT_voyager(
    KC_TILD,        KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_GRAVE,       KC_EXLM,        KC_AT,          KC_LPRN,        KC_RPRN,        KC_PERC,                                        KC_TRANSPARENT, KC_SLASH,       KC_MINUS,       KC_GRAVE,       KC_TRANSPARENT, KC_F12,         
    KC_TRANSPARENT, KC_MINUS,       KC_PLUS,        KC_LBRC,        KC_RBRC,        ST_MACRO_0,                                     KC_TRANSPARENT, KC_DQUO,        KC_EQUAL,       KC_PLUS,        KC_SCLN,        TO(0),          
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_LCBR,        KC_RCBR,        KC_SCLN,                                        KC_TRANSPARENT, KC_UNDS,        KC_COMMA,       KC_DOT,         KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, TO(0)
  ),
  [2] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,        
    KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_UP,          KC_MEDIA_PLAY_PAUSE,LGUI(KC_GRAVE),                                 KC_PAGE_UP,     KC_HOME,        KC_TRANSPARENT, KC_END,         KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_LEFT,        KC_DOWN,        KC_RIGHT,       LGUI(KC_H),                                     KC_PGDN,        ST_MACRO_1,     ST_MACRO_2,     ST_MACRO_3,     KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_AUDIO_MUTE,  LGUI(LSFT(KC_4)),LGUI(LCTL(LSFT(KC_4))),LGUI(KC_TAB),   LGUI(LCTL(KC_T)),                                KC_TRANSPARENT, LGUI(LSFT(KC_LBRC)),LGUI(LSFT(KC_RBRC)),KC_TRANSPARENT, KC_TRANSPARENT, TO(0),          
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, TO(0)
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_ASTR,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_EQUAL,       KC_PLUS,        KC_MINUS,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_COLN,        KC_PERC,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, TO(0)
  ),
};

const uint16_t PROGMEM combo0[] = { MT(MOD_RGUI, KC_J), MT(MOD_RSFT, KC_K), COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_ESCAPE),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LT(2,KC_BSPC):
            return TAPPING_TERM -100;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {89,255,255}, {0,245,183}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {139,213,250}, {0,0,255} },

    [1] = { {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204}, {0,218,204} },

    [2] = { {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {132,183,251}, {16,200,255}, {73,218,204}, {73,218,204}, {73,218,204}, {132,183,251}, {132,183,251}, {132,183,251}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204}, {73,218,204} },

    [3] = { {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157}, {183,204,157} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_EQUAL) SS_DELAY(100) SS_LSFT(SS_TAP(X_DOT)));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_X)) SS_DELAY(100) SS_TAP(X_0));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_X)) SS_DELAY(100) SS_TAP(X_1));
    }
    break;
    case ST_MACRO_3:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_X)) SS_DELAY(100) SS_TAP(X_O));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[4];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_X);
        tap_code16(KC_X);
        tap_code16(KC_X);
    }
    if(state->count > 3) {
        tap_code16(KC_X);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_X); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_X)); break;
        case DOUBLE_TAP: register_code16(KC_X); register_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_X); register_code16(KC_X);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_X); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_X)); break;
        case DOUBLE_TAP: unregister_code16(KC_X); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_X); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_C);
        tap_code16(KC_C);
        tap_code16(KC_C);
    }
    if(state->count > 3) {
        tap_code16(KC_C);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_C); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_C)); break;
        case DOUBLE_TAP: register_code16(KC_C); register_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_C); register_code16(KC_C);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_C); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_C)); break;
        case DOUBLE_TAP: unregister_code16(KC_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_C); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_V);
        tap_code16(KC_V);
        tap_code16(KC_V);
    }
    if(state->count > 3) {
        tap_code16(KC_V);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_V); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: register_code16(KC_V); register_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_V); register_code16(KC_V);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_V); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: unregister_code16(KC_V); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_V); break;
    }
    dance_state[2].step = 0;
}
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case DOUBLE_TAP: layer_move(2); break;
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
    }
    dance_state[3].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_3_finished, dance_3_reset),
};
