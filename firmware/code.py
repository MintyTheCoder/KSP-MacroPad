import board
import analogio
import time
import rotaryio

import usb_cdc

serial = usb_cdc.data

IDLE_MAX = 0.3
LOW_MIN = 0.7
LOW_MAX = 1.3
HIGH_MIN = 1.6
HIGH_MAX = 2.3

DEBOUNCE_COUNT = 4

COUNTS_PER_DETENT = 4


_stable_key = None
_stable_count = 0
_reported_key = None

_left_accum = 0
_right_accum = 0


row_pins = [analogio.AnalogIn(board.A0), analogio.AnalogIn(board.A1)]
col_pins = [analogio.AnalogIn(board.A2), analogio.AnalogIn(board.A3)]

left_enc = rotaryio.IncrementalEncoder(board.D10, board.D9)
right_enc = rotaryio.IncrementalEncoder(board.D7, board.D8)

_left_last = left_enc.position
_right_last = right_enc.position

KEY_MAP = [
    [0x00, 0x04, 0x08, 0x0C],
    [0x01, 0x05, 0x09, 0x0D],
    [0x02, 0x06, 0x0A, 0x0E],
    [0x03, 0x07, 0x0B, 0x0F],
]

def read_voltage(pin):
    return pin.value / 65535 * 3.3

def classify(voltage):
    if voltage < IDLE_MAX:
        return None
    if LOW_MIN <= voltage <= LOW_MAX:
        return 0
    if HIGH_MIN <= voltage <= HIGH_MAX:
        return 1
    return None

def find_index(pins):
    for pin_idx, pin in enumerate(pins):
        level = classify(read_voltage(pin))
        if level is not None:
            return pin_idx * 2 + level
    return None

def read_key():
    row = find_index(row_pins)
    col = find_index(col_pins)
    if row is None or col is None:
        return None
    return KEY_MAP[row][col]


LED_CHAIN_TO_KEY = [
    None,
    0x00, 0x04, 0x08, 0x0C,
    None,
    0x0D, 0x09, 0x05, 0x01,
    0x0E, 0x0A, 0x06, 0x02,
    0x03, 0x07, 0x0B, 0x0F,
    None, None,
]

KEY_TO_LED_CHAIN = {key: i for i, key in enumerate(LED_CHAIN_TO_KEY) if key is not None}

UNDERGLOW_CHAIN_INDICES = [i for i, key in enumerate(LED_CHAIN_TO_KEY) if key is None]

def poll_key():
    global _stable_key, _stable_count, _reported_key

    current = read_key()

    if current == _stable_key:
        _stable_count += 1
    else:
        _stable_key = current
        _stable_count = 1

    if _stable_count < DEBOUNCE_COUNT:
        return None

    if _stable_key != _reported_key:
        _reported_key = _stable_key
        if _stable_key is not None:
            return _stable_key

    return None

def poll_encoders():
    global _left_last, _right_last, _left_accum, _right_accum

    left_pos = left_enc.position
    right_pos = right_enc.position

    _left_accum += left_pos - _left_last
    _right_accum += right_pos - _right_last

    _left_last = left_pos
    _right_last = right_pos

    left_steps = _left_accum // COUNTS_PER_DETENT
    right_steps = _right_accum // COUNTS_PER_DETENT

    _left_accum -= left_steps * COUNTS_PER_DETENT
    _right_accum -= right_steps * COUNTS_PER_DETENT

    return left_steps, right_steps

def send_key_packet(key_id):
    packet = bytes([0x44, 0x01, key_id, 0x00, 0x01, 0x77])
    serial.write(packet)

def send_encoder_packet(encoder_id, steps):
    steps = max(-32768, min(32767, steps))
    hi = (steps >> 8) & 0xFF
    lo = steps & 0xFF
    packet = bytes([0x44, 0x02, encoder_id, hi, lo, 0x77])
    serial.write(packet)