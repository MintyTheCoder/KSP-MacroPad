import board
import analogio

IDLE_MAX = 0.3
LOW_MIN = 0.7
LOW_MAX = 1.3
HIGH_MIN = 1.6
HIGH_MAX = 2.3

row_pins = [analogio.AnalogIn(board.A0), analogio.AnalogIn(board.A1)]
col_pins = [analogio.AnalogIn(board.A2), analogio.AnalogIn(board.A3)]

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