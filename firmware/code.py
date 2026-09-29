# ============================================================================
# KSP MacroPad — Firmware (CircuitPython)
# Version: 0.3.0
# ============================================================================

import time

import board
import analogio
import digitalio
import busio
import rotaryio
import neopixel
import usb_cdc
import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_ssd1306

from led_states import LEDStates

serial = usb_cdc.data
serial.write_timeout = 0.01   # default is None (block forever), which freezes the pad if nothing reads the port


# ----------------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------------

# Key matrix (see read_key). Each column pin (A2, A3) feeds two column lines:
# one directly (0 ohm), one through 10k. Each row pin (A0, A1) reads two row
# lines, one directly and one through 16k, into a 10k pulldown. With a
# column pin driven high, a pressed key puts one of four voltages on its row
# pin (1% resistors, 1N4148 drop 0.48-0.65 V, ADC error included):
#   10k column + 16k row   0.70 - 0.82 V
#   0R  column + 16k row   0.98 - 1.13 V
#   10k column + 0R  row   1.28 - 1.45 V
#   0R  column + 0R  row   2.62 - 2.85 V
MATRIX_IDLE_MAX = 0.40
MATRIX_BANDS = (   # (upper edge in volts, column line, row line); line 0 = direct, 1 = through a resistor
    (0.90, 1, 1),
    (1.205, 0, 1),
    (2.00, 1, 0),
    (3.40, 0, 0),
)

DEBOUNCE_COUNT = 4      # consecutive stable reads required before a key registers
COUNTS_PER_DETENT = 4   # raw encoder counts per physical detent click
NUM_LEDS = 20           # total pixels in the NeoPixel chain

LOOP_INTERVAL = 0.01    # seconds per main loop tick

# SUICIDE BURN ARM is the one macro that needs a safety hold, not a tap:
# a plain press/release never fires it, only a continuous hold does.
SUICIDE_ARM_KEY = 0x0A
SUICIDE_ARM_HOLD_SECONDS = 1.0
SUICIDE_ARM_HOLD_TICKS = int(SUICIDE_ARM_HOLD_SECONDS / LOOP_INTERVAL)

OLED_WIDTH = 128
OLED_HEIGHT = 32
OLED_TEXT_COLUMNS = OLED_WIDTH // 6   # terminalio.FONT is 6px wide -> 21 columns

# Boot animation: T-3/2/1 countdown, then a rocket climbing off the top of
# the screen, then the title. LEDs fill along the chain during each part.
BOOT_COUNTDOWN_STEP_SECONDS = 0.35
BOOT_LAUNCH_FRAME_SECONDS = 0.04
BOOT_LAUNCH_PIXELS_PER_FRAME = 2
BOOT_TITLE_SECONDS = 0.8
BOOT_COUNTDOWN_COLOR = (255, 120, 0)
BOOT_LAUNCH_COLOR = (0, 100, 255)

# '#' = rocket body, '*' = exhaust (flickers every frame), '.' = empty
ROCKET_ART = (
    "...##...",
    "..####..",
    "..#..#..",
    "..####..",
    "..####..",
    "..####..",
    ".######.",
    "##.##.##",
    "#..##..#",
    "...**...",
    "..*..*..",
    "...**...",
)

# Reserved inbound IDs outside the key (0x00-0x0F) / underglow (0x10-0x13)
# ranges — same 5-byte packet frame, just new meanings for id/state/data.
HEARTBEAT_ID = 0x14           # mod->pad, sent periodically regardless of state change
THROTTLE_TELEMETRY_ID = 0x15  # data = live throttle % (0-100)
WARP_TELEMETRY_ID = 0x16      # data = live warp index (mod-defined 0-255 lookup)
ACTIVE_MACRO_ID = 0x17        # data = key id of the macro flying the vessel, NO_MACRO if none
NEXT_MACRO_ID = 0x18          # data = key id at the head of the mod's queue, NO_MACRO if empty
QUEUE_LENGTH_ID = 0x19        # data = number of queued macros (not counting the active one)
NO_MACRO = 0xFF

# 4-char OLED tags for the macros that can be active/queued.
MACRO_TAGS = {
    0x01: "AGT",
    0x02: "CIRC",
    0x04: "INTC",
    0x05: "SYNC",
    0x07: "DORB",
    0x0A: "SBRN",
    0x0F: "AUTO",
}

HEARTBEAT_TIMEOUT = 3.0   # seconds since last heartbeat before we call it disconnected

# Pad -> mod hello, sent periodically so the mod can tell this data port
# apart from the REPL console port (and anything else plugged in).
PAD_HELLO_TYPE = 0x03
PAD_HELLO_INTERVAL = 1.0
PAD_RESYNC_TYPE = 0x04   # "resend everything" - sent when the heartbeat comes back after a lapse


# ----------------------------------------------------------------------------
# Hardware setup
# ----------------------------------------------------------------------------

row_pins = [analogio.AnalogIn(board.A0), analogio.AnalogIn(board.A1)]
col_drives = [digitalio.DigitalInOut(board.A2), digitalio.DigitalInOut(board.A3)]   # high-Z until scanned

left_enc = rotaryio.IncrementalEncoder(board.D10, board.D9)
right_enc = rotaryio.IncrementalEncoder(board.D7, board.D8)

pixels = neopixel.NeoPixel(board.D6, NUM_LEDS, auto_write=False)

displayio.release_displays()
_i2c = busio.I2C(board.D5, board.D4)   # SCL, SDA - the PCB routes the OLED to D5/D4 (D0/D1 are A0/A1, the matrix)
_display_bus = displayio.I2CDisplay(_i2c, device_address=0x3C)
display = adafruit_displayio_ssd1306.SSD1306(_display_bus, width=OLED_WIDTH, height=OLED_HEIGHT)


# ----------------------------------------------------------------------------
# Runtime state
# ----------------------------------------------------------------------------

encoder_mode = 1   # 1 = normal, 2 = precision input, 3 = aux mode

# Locally-dialed target values for modes 2/3 (pre-commit, display-only —
# not the mod's authoritative state). Indexed by mode.
dial_targets = {
    2: {"left": 0, "right": 0},   # left = target throttle % (0-100), right = target heading deg (0-359)
    3: {"left": 0, "right": 0},   # left = camera zoom (unclamped), right = RCS thrust limiter % (0-100)
}

# Mode 1 live telemetry, confirmed by the mod (see THROTTLE/WARP_TELEMETRY_ID
# below) — None until the first packet of that type arrives.
live_throttle = None
live_warp = None

# Macro queue as reported by the mod (see ACTIVE_MACRO_ID etc.). Cleared
# on disconnect since the mod can't be flying anything we can see.
active_macro = None
next_macro = None
queued_count = 0

_last_heartbeat = None      # time.monotonic() of the last heartbeat, None = never seen one
_last_connected = False     # last displayed connection state, so update_display only fires on change

_stable_key = None
_stable_count = 0
_reported_key = None

_left_accum = 0
_right_accum = 0
_left_last = left_enc.position
_right_last = right_enc.position

_in_buf = bytearray()
_last_hello = 0.0


# ----------------------------------------------------------------------------
# Key matrix mapping
# ----------------------------------------------------------------------------
# Key ids run left to right, top to bottom: the top row (nearest the OLED) is
# 0x00-0x03, the bottom row 0x0C-0x0F. read_key returns row * 4 + col.


# ----------------------------------------------------------------------------
# LED chain mapping
# ----------------------------------------------------------------------------
# Physical NeoPixel chain order -> key_id it lights up for.
# None = underglow pixel (not tied to a specific key).
# The chain snakes across the rows: top row left to right, second row right
# to left, and so on, with the corner underglow LEDs at positions 0, 5, 18, 19.
LED_CHAIN_TO_KEY = [
    None,
    0x00, 0x01, 0x02, 0x03,
    None,
    0x07, 0x06, 0x05, 0x04,
    0x08, 0x09, 0x0A, 0x0B,
    0x0F, 0x0E, 0x0D, 0x0C,
    None, None,
]

# Reverse lookup: key_id -> chain index
KEY_TO_LED_CHAIN = {key: i for i, key in enumerate(LED_CHAIN_TO_KEY) if key is not None}

# Chain indices reserved for underglow (not mapped to any key)
UNDERGLOW_CHAIN_INDICES = [i for i, key in enumerate(LED_CHAIN_TO_KEY) if key is None]


# ----------------------------------------------------------------------------
# LED state -> color table
# ----------------------------------------------------------------------------
# Keyed by (led_id, state). led_id 0x10-0x13 (underglow zones) all share the
# 0x10 entries — see apply_led_update's lookup_id collapsing.
STATE_COLORS = {
    (0x00, LEDStates.LAUNCH_IDLE): (0, 0, 0),
    (0x00, LEDStates.LAUNCH_EXECUTING): (0, 100, 255),
    (0x00, LEDStates.LAUNCH_COMPLETE): (0, 255, 0),

    (0x01, LEDStates.AGT_IDLE): (0, 0, 0),
    (0x01, LEDStates.AGT_ACTIVE): (0, 100, 255),

    (0x02, LEDStates.CIRC_IDLE): (0, 0, 0),
    (0x02, LEDStates.CIRC_UNAVAILABLE): (255, 0, 0),
    (0x02, LEDStates.CIRC_CALCULATING): (255, 180, 0),
    (0x02, LEDStates.CIRC_WARPING): (0, 255, 255),
    (0x02, LEDStates.CIRC_COMPLETE): (0, 255, 0),

    (0x03, LEDStates.TIMEACCEL_IDLE): (0, 0, 0),
    (0x03, LEDStates.TIMEACCEL_UNAVAILABLE): (255, 0, 0),
    (0x03, LEDStates.TIMEACCEL_WARPING): (0, 255, 255),

    (0x04, LEDStates.INTERCEPT_IDLE): (0, 0, 0),
    (0x04, LEDStates.INTERCEPT_CALCULATING): (255, 180, 0),
    (0x04, LEDStates.INTERCEPT_SOLUTION): (0, 255, 0),
    (0x04, LEDStates.INTERCEPT_INSUFFICIENT_DV): (255, 0, 0),

    (0x05, LEDStates.ORBSYNC_IDLE): (0, 0, 0),
    (0x05, LEDStates.ORBSYNC_CALCULATING): (255, 180, 0),
    (0x05, LEDStates.ORBSYNC_EXECUTING): (0, 100, 255),
    (0x05, LEDStates.ORBSYNC_COMPLETE): (0, 255, 0),

    (0x06, LEDStates.RENDEZVOUS_IDLE): (0, 0, 0),
    (0x06, LEDStates.RENDEZVOUS_ACTIVE): (0, 100, 255),
    (0x06, LEDStates.RENDEZVOUS_IN_RANGE): (0, 255, 0),

    (0x07, LEDStates.DEORBIT_IDLE): (0, 0, 0),
    (0x07, LEDStates.DEORBIT_UNAVAILABLE): (255, 0, 0),
    (0x07, LEDStates.DEORBIT_PLANNED): (255, 180, 0),
    (0x07, LEDStates.DEORBIT_WARPING): (0, 255, 255),
    (0x07, LEDStates.DEORBIT_BURNING): (0, 100, 255),

    (0x08, LEDStates.DOCK_IDLE): (0, 0, 0),
    (0x08, LEDStates.DOCK_ACTIVE): (0, 100, 255),
    (0x08, LEDStates.DOCK_TARGET_ACQUIRED): (0, 200, 255),
    (0x08, LEDStates.DOCK_READY): (0, 255, 0),

    (0x09, LEDStates.LANDING_IDLE): (0, 0, 0),
    (0x09, LEDStates.LANDING_CONFIGURING): (255, 180, 0),
    (0x09, LEDStates.LANDING_COMPLETE): (0, 255, 0),

    (0x0A, LEDStates.SUICIDEBURN_IDLE): (0, 0, 0),
    (0x0A, LEDStates.SUICIDEBURN_ARMED): (255, 90, 0),
    (0x0A, LEDStates.SUICIDEBURN_IMMINENT): (255, 0, 0),
    (0x0A, LEDStates.SUICIDEBURN_BURNING): (0, 100, 255),

    (0x0C, LEDStates.SCIENCE_IDLE): (0, 0, 0),
    (0x0C, LEDStates.SCIENCE_UNAVAILABLE): (255, 0, 0),
    (0x0C, LEDStates.SCIENCE_TRANSMITTING): (0, 100, 255),
    (0x0C, LEDStates.SCIENCE_COMPLETE): (0, 255, 0),

    (0x0D, LEDStates.RESOURCE_IDLE): (0, 0, 0),
    (0x0D, LEDStates.RESOURCE_NOMINAL): (0, 255, 0),
    (0x0D, LEDStates.RESOURCE_LOW): (255, 180, 0),
    (0x0D, LEDStates.RESOURCE_VERYLOW): (255, 90, 0),
    (0x0D, LEDStates.RESOURCE_CRITICAL): (255, 0, 0),
    (0x0D, LEDStates.RESOURCE_DEPLETING): (255, 0, 0),

    (0x0F, LEDStates.AUTOPILOT_IDLE): (0, 0, 0),
    (0x0F, LEDStates.AUTOPILOT_IMPOSSIBLE): (255, 0, 0),
    (0x0F, LEDStates.AUTOPILOT_READY): (0, 255, 0),
    (0x0F, LEDStates.AUTOPILOT_PLANNING): (255, 180, 0),
    (0x0F, LEDStates.AUTOPILOT_EXECUTING): (0, 100, 255),

    (0x10, LEDStates.UNDERGLOW_IDLE): (0, 0, 0),
    (0x10, LEDStates.UNDERGLOW_CONNECTED_NOVESSEL): (40, 40, 40),
    (0x10, LEDStates.UNDERGLOW_LAUNCHPAD): (255, 180, 0),
    (0x10, LEDStates.UNDERGLOW_ASCENT): (0, 100, 255),
    (0x10, LEDStates.UNDERGLOW_STABLE_ORBIT): (0, 255, 0),
    (0x10, LEDStates.UNDERGLOW_TIMEWARP): (0, 255, 255),
    (0x10, LEDStates.UNDERGLOW_MANEUVER_IMMINENT): (255, 90, 0),
    (0x10, LEDStates.UNDERGLOW_LANDING): (255, 180, 0),
    (0x10, LEDStates.UNDERGLOW_AUTOPILOT): (150, 0, 255),
    (0x10, LEDStates.UNDERGLOW_CRITICAL_RESOURCE): (255, 0, 0),
    (0x10, LEDStates.UNDERGLOW_COMMS_LOST): (255, 0, 0),
}


# ----------------------------------------------------------------------------
# Key matrix scanning
# ----------------------------------------------------------------------------

def read_voltage(pin):
    return pin.value / 65535 * 3.3


def decode_matrix(col_pin, row_pin, voltage):
    """(row, col) of the key closed between the driven col_pin and row_pin,
    from that row pin's voltage, or None if nothing is pressed there."""
    if voltage < MATRIX_IDLE_MAX:
        return None
    for edge, col_line, row_line in MATRIX_BANDS:
        if voltage < edge:
            return row_pin * 2 + row_line, col_pin * 2 + col_line
    return None


def read_key():
    """Drive each column pin high in turn and read both row pins. The matrix
    has no other voltage source, so an undriven column pin reads nothing.
    Single keys only - two keys on the same row pin under the same driven
    column add their currents and read as a different key."""
    found = None
    for c, drive in enumerate(col_drives):
        drive.switch_to_output(value=True)
        for r, pin in enumerate(row_pins):
            hit = decode_matrix(c, r, read_voltage(pin))
            if hit is not None and found is None:
                found = hit
        drive.switch_to_input()
    if found is None:
        return None
    row, col = found
    return row * 4 + col


def poll_key():
    """Debounced key read. Returns a key_id only on a new, stable press.

    SUICIDE_ARM_KEY is the exception: it needs SUICIDE_ARM_HOLD_TICKS of
    continuous hold (not just DEBOUNCE_COUNT) before it fires. Releasing
    early resets _stable_count back to 0 on the next differing read, so
    an early release simply never reaches the threshold — no separate
    cancel signal needed."""
    global _stable_key, _stable_count, _reported_key

    current = read_key()

    if current == _stable_key:
        _stable_count += 1
    else:
        _stable_key = current
        _stable_count = 1

    required = SUICIDE_ARM_HOLD_TICKS if _stable_key == SUICIDE_ARM_KEY else DEBOUNCE_COUNT
    if _stable_count < required:
        return None

    if _stable_key != _reported_key:
        _reported_key = _stable_key
        if _stable_key is not None:
            return _stable_key

    return None


# ----------------------------------------------------------------------------
# Encoder polling
# ----------------------------------------------------------------------------

def poll_encoders():
    """Accumulate raw encoder counts and emit whole detent steps."""
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


# ----------------------------------------------------------------------------
# Serial protocol — outbound (pad -> mod)
# ----------------------------------------------------------------------------
# Packet format: [0x44, msg_type, id, data_hi, data_lo, 0x77]

def send_packet(packet):
    """Write only when a host has the port open (DTR set); otherwise the
    bytes have nowhere to go."""
    if serial.connected:
        serial.write(packet)


def send_key_packet(key_id):
    send_packet(bytes([0x44, 0x01, key_id, 0x00, 0x01, 0x77]))


def send_hello():
    send_packet(bytes([0x44, PAD_HELLO_TYPE, 0x00, 0x00, 0x00, 0x77]))


def send_resync():
    send_packet(bytes([0x44, PAD_RESYNC_TYPE, 0x00, 0x00, 0x00, 0x77]))


def send_encoder_packet(encoder_id, steps):
    steps = max(-32768, min(32767, steps))
    hi = (steps >> 8) & 0xFF
    lo = steps & 0xFF
    send_packet(bytes([0x44, 0x02, encoder_id, hi, lo, 0x77]))


# ----------------------------------------------------------------------------
# Serial protocol — inbound (mod -> pad)
# ----------------------------------------------------------------------------
# Packet format: [0x77, id, state, data, 0x44] — shared frame for LED
# updates, telemetry (THROTTLE/WARP_TELEMETRY_ID), and HEARTBEAT_ID.

def read_inbound_packet():
    """Pull one complete inbound packet off the serial buffer, if available."""
    global _in_buf
    if serial.in_waiting > 0:
        _in_buf += serial.read(serial.in_waiting)

    while len(_in_buf) >= 5:
        if _in_buf[0] != 0x77:
            _in_buf = _in_buf[1:]
            continue
        if _in_buf[4] != 0x44:
            _in_buf = _in_buf[1:]
            continue
        packet_id, state, data = _in_buf[1], _in_buf[2], _in_buf[3]
        _in_buf = _in_buf[5:]
        return packet_id, state, data

    return None


def apply_led_update(led_id, state, data):
    """Resolve an LED packet to a color and write it to the pixel chain."""
    lookup_id = 0x10 if 0x10 <= led_id <= 0x13 else led_id
    color = STATE_COLORS.get((lookup_id, state), (0, 0, 0))

    if led_id in KEY_TO_LED_CHAIN:
        pixels[KEY_TO_LED_CHAIN[led_id]] = color
    elif 0x10 <= led_id <= 0x13:
        pixels[UNDERGLOW_CHAIN_INDICES[led_id - 0x10]] = color
    else:
        return

    pixels.show()


# ----------------------------------------------------------------------------
# OLED display
# ----------------------------------------------------------------------------
# Line 1: connection status (from HEARTBEAT_ID) + current encoder mode.
# Line 2: mode 1 shows mod-confirmed live throttle/warp (THROTTLE/WARP_
# TELEMETRY_ID) once received; modes 2/3 show the locally-dialed target
# values (pre-commit, not yet sent to the mod as a "final" value).

def _fill_led_chain(fraction, color):
    """Light the first `fraction` of the chain (in chain order) with color."""
    lit = int(NUM_LEDS * fraction)
    for i in range(NUM_LEDS):
        pixels[i] = color if i < lit else (0, 0, 0)
    pixels.show()


def _make_rocket():
    """Build the rocket TileGrid from ROCKET_ART. Returns (tilegrid, palette);
    palette[2] is the exhaust color, toggled per frame for flicker."""
    height = len(ROCKET_ART)
    width = len(ROCKET_ART[0])
    bitmap = displayio.Bitmap(width, height, 3)
    palette = displayio.Palette(3)
    palette[0] = 0x000000
    palette[1] = 0xFFFFFF
    palette[2] = 0xFFFFFF
    palette.make_transparent(0)
    for y, row in enumerate(ROCKET_ART):
        for x, ch in enumerate(row):
            if ch == "#":
                bitmap[x, y] = 1
            elif ch == "*":
                bitmap[x, y] = 2
    return displayio.TileGrid(bitmap, pixel_shader=palette), palette


def show_boot_animation():
    group = displayio.Group()
    display.root_group = group

    # T-3, T-2, T-1 at 2x scale (12px per char), LEDs filling amber.
    countdown = label.Label(terminalio.FONT, text="", scale=2, x=(OLED_WIDTH - 3 * 12) // 2, y=16)
    group.append(countdown)
    for i, n in enumerate((3, 2, 1)):
        countdown.text = "T-{}".format(n)
        _fill_led_chain((i + 1) / 3, BOOT_COUNTDOWN_COLOR)
        time.sleep(BOOT_COUNTDOWN_STEP_SECONDS)
    group.remove(countdown)

    # Liftoff: rocket climbs from below the screen until it's off the top,
    # LEDs refilling blue along the chain as it goes.
    rocket, palette = _make_rocket()
    rocket.x = (OLED_WIDTH - len(ROCKET_ART[0])) // 2
    group.append(rocket)
    positions = list(range(OLED_HEIGHT, -len(ROCKET_ART) - 1, -BOOT_LAUNCH_PIXELS_PER_FRAME))
    for frame, y in enumerate(positions):
        rocket.y = y
        palette[2] = 0xFFFFFF if frame % 2 == 0 else 0x000000
        _fill_led_chain((frame + 1) / len(positions), BOOT_LAUNCH_COLOR)
        time.sleep(BOOT_LAUNCH_FRAME_SECONDS)
    group.remove(rocket)

    title_text = "KSP MACROPAD"
    group.append(label.Label(terminalio.FONT, text=title_text, x=(OLED_WIDTH - len(title_text) * 6) // 2, y=16))
    time.sleep(BOOT_TITLE_SECONDS)

    # Hand the LEDs back blank; the mod sets real states once connected.
    pixels.fill((0, 0, 0))
    pixels.show()


line1 = label.Label(terminalio.FONT, text="", x=0, y=8)
line2 = label.Label(terminalio.FONT, text="", x=0, y=24)

main_group = displayio.Group()
main_group.append(line1)
main_group.append(line2)


def format_macro_tag(active, nxt, count):
    """Right side of line 1: "" when idle, "CIRC" when one macro is flying,
    "CIRC>SYNC" with one queued, "CIRC>SYNC+2" with three queued. Worst
    case ("AUTO>DORB+9") is 11 chars, which fits beside "CONN M1"; it's
    only shown while connected, so it never has to fit beside "NO CONN"."""
    if active is None:
        return ""
    tag = MACRO_TAGS.get(active, "????")
    if nxt is not None:
        tag += ">" + MACRO_TAGS.get(nxt, "????")
        if count > 1:
            tag += "+{}".format(count - 1)
    return tag


def update_display():
    """Refresh both OLED lines from current mode/dial/telemetry/connection
    state. Call only on a state change (mode toggle, nonzero encoder step,
    telemetry packet, or a connection-status flip) — not every loop tick,
    since each text assignment triggers a redraw."""
    conn_text = "CONN" if _last_connected else "NO CONN"
    left = "{} M{}".format(conn_text, encoder_mode)
    tag = format_macro_tag(active_macro, next_macro, queued_count) if _last_connected else ""
    padding = max(1, OLED_TEXT_COLUMNS - len(left) - len(tag)) if tag else 0
    line1.text = left + " " * padding + tag

    if encoder_mode == 1:
        if live_throttle is None or live_warp is None:
            line2.text = "--"   # no telemetry received yet
        else:
            line2.text = "THR {:>3}% WARP {}".format(live_throttle, live_warp)
    elif encoder_mode == 2:
        t = dial_targets[2]
        line2.text = "THR {:>3}% HDG {:>3}".format(t["left"], t["right"])
    elif encoder_mode == 3:
        t = dial_targets[3]
        line2.text = "ZOOM {:>3} RCS {:>3}%".format(t["left"], t["right"])


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------

show_boot_animation()
display.root_group = main_group
update_display()

while True:
    key = poll_key()
    if key is not None:
        send_key_packet(key)
        # 0x0B / 0x0E toggle their own mode on/off (2 or 3), returning to
        # mode 1 on a second press.
        if key == 0x0B:
            encoder_mode = 1 if encoder_mode == 2 else 2
            update_display()
        elif key == 0x0E:
            encoder_mode = 1 if encoder_mode == 3 else 3
            update_display()

    left_steps, right_steps = poll_encoders()
    if left_steps != 0:
        send_encoder_packet((encoder_mode << 4) | 0x1, left_steps)
        if encoder_mode == 2:
            t = dial_targets[2]
            t["left"] = max(0, min(100, t["left"] + left_steps))
            update_display()
        elif encoder_mode == 3:
            dial_targets[3]["left"] += left_steps   # zoom: unclamped
            update_display()

    if right_steps != 0:
        send_encoder_packet((encoder_mode << 4) | 0x2, right_steps)
        if encoder_mode == 2:
            t = dial_targets[2]
            t["right"] = (t["right"] + right_steps) % 360
            update_display()
        elif encoder_mode == 3:
            t = dial_targets[3]
            t["right"] = max(0, min(100, t["right"] + right_steps))
            update_display()

    inbound = read_inbound_packet()
    if inbound is not None:
        packet_id, state, data = inbound
        if packet_id == HEARTBEAT_ID:
            _last_heartbeat = time.monotonic()
        elif packet_id == THROTTLE_TELEMETRY_ID:
            live_throttle = data
            if encoder_mode == 1:
                update_display()
        elif packet_id == WARP_TELEMETRY_ID:
            live_warp = data
            if encoder_mode == 1:
                update_display()
        elif packet_id == ACTIVE_MACRO_ID:
            active_macro = None if data == NO_MACRO else data
            update_display()
        elif packet_id == NEXT_MACRO_ID:
            next_macro = None if data == NO_MACRO else data
            update_display()
        elif packet_id == QUEUE_LENGTH_ID:
            queued_count = data
            update_display()
        else:
            apply_led_update(packet_id, state, data)

    # Connection status can lapse without a new packet ever arriving, so
    # check the heartbeat timeout every tick, not just on packet receipt.
    connected = _last_heartbeat is not None and (time.monotonic() - _last_heartbeat) < HEARTBEAT_TIMEOUT
    if connected != _last_connected:
        _last_connected = connected
        if not connected:
            active_macro = None
            next_macro = None
            queued_count = 0
        else:
            send_resync()
        update_display()

    now = time.monotonic()
    if now - _last_hello >= PAD_HELLO_INTERVAL:
        _last_hello = now
        send_hello()

    time.sleep(LOOP_INTERVAL)
