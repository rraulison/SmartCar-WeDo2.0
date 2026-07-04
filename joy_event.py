#!/usr/bin/env python3
"""
Controle do LEGO WeDo 2.0 SmartCar com joystick Bluetooth.

Suporta dois modos de entrada:
  - joydev (/dev/input/js*) — quando o kernel tem o modulo joydev
  - evdev  (/dev/input/event*) — leitura direta de eventos do kernel

Modo arcade com analogico ESQUERDO (invertido):
  - Y para cima  -> motores A e B tras (reverso)
  - Y para baixo -> motores A e B frente
  - X para direita -> Motor B frente, Motor A tras (vira esquerda)
  - X para esquerda -> Motor A frente, Motor B tras (vira direita)

Tecla Q (ou Ctrl+C): encerra o programa.
"""

import argparse
import asyncio
import fcntl
import glob
import os
import select
import struct
import sys
import termios
import threading
import time
import traceback
import tty
from dataclasses import dataclass, field

from bleak import BleakClient, BleakScanner

# ---------------------------------------------------------------------------
# Constantes BLE / WeDo 2.0
# ---------------------------------------------------------------------------
HUB_ADDRESS = "74:8B:34:94:63:3B"
HUB_NAME = "M_SmartCar_2/0"

CHAR_WRITE = "00001565-1212-efde-1523-785feabcd123"
CHAR_LOW_VOLTAGE = "00001528-1212-efde-1523-785feabcd123"
CHAR_HIGH_CURRENT = "00001529-1212-efde-1523-785feabcd123"

PORT_A = 0x01
PORT_B = 0x02

INVERT_A = False
INVERT_B = True

MIN_POWER = 55
MAX_POWER = 100
SAFE_MOTOR_LEVELS = (0, 55, 100)
DEAD_ZONE = 0.12
SMOOTHING = 0.35

BLE_INTERVAL = 0.06
POWER_BUDGET = 200
DUAL_START_STAGGER = 0.25
POWER_ALERT_COOLDOWN = 2.0
ENABLE_POWER_ALERTS = True
MOTOR_SELF_TEST = True
SELF_TEST_POWER = 55
SELF_TEST_SECONDS = 0.50
DITHER_MODE = "sigma"
DITHER_SPREAD_TICKS = 16

# ---------------------------------------------------------------------------
# Constantes de entrada — joydev (/dev/input/js*)
# ---------------------------------------------------------------------------
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_SIZE = 8

# ---------------------------------------------------------------------------
# Constantes de entrada — evdev (/dev/input/event*)
# ---------------------------------------------------------------------------
EV_KEY = 0x01
EV_ABS = 0x03
ABS_X = 0
ABS_Y = 1
ABS_RX = 3
ABS_RY = 4
BTN_SOUTH = 304

# struct input_event: long sec, long usec, unsigned short type, unsigned short code, signed int value
_INPUT_EVENT_FMT = "llHHi"
INPUT_EVENT_SIZE = struct.calcsize(_INPUT_EVENT_FMT)

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_READ = 2


# ---------------------------------------------------------------------------
# Estado compartilhado entre threads
# ---------------------------------------------------------------------------
@dataclass
class SharedState:
    speed_a: int = 0
    speed_b: int = 0
    connected: bool = False
    running: bool = True
    low_voltage: bool = False
    high_current: bool = False
    cooldown_until: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)


state = SharedState()


# ---------------------------------------------------------------------------
# Motor helpers
# ---------------------------------------------------------------------------
def clamp_speed(speed: int) -> int:
    return max(-100, min(100, int(speed)))


def quantize_motor_speed(speed: int) -> int:
    """Usa apenas os niveis que o hub/motores aceitaram no teste local."""
    speed = clamp_speed(speed)
    if speed == 0:
        return 0
    sign = 1 if speed > 0 else -1
    magnitude = abs(speed)
    level = SAFE_MOTOR_LEVELS[1] if magnitude < 78 else SAFE_MOTOR_LEVELS[2]
    return sign * level


def effective_to_safe_pair(speed: int) -> tuple[int, int, float]:
    """Converte uma velocidade desejada em dois comandos seguros e duty."""
    speed = clamp_speed(speed)
    if speed == 0:
        return 0, 0, 0.0
    sign = 1 if speed > 0 else -1
    magnitude = abs(speed)
    safe_low = SAFE_MOTOR_LEVELS[1]
    safe_high = SAFE_MOTOR_LEVELS[2]
    if magnitude <= safe_low:
        return 0, sign * safe_low, magnitude / safe_low
    duty_high = (magnitude - safe_low) / (safe_high - safe_low)
    return sign * safe_low, sign * safe_high, duty_high


class MotorDither:
    def __init__(self, mode: str) -> None:
        self.mode = mode
        self.accumulator = 0.0
        self.polarity = 0

    def reset_if_needed(self, speed: int) -> None:
        polarity = 0 if speed == 0 else (1 if speed > 0 else -1)
        if polarity != self.polarity:
            self.accumulator = 0.0
            self.polarity = polarity

    def next_command(self, speed: int) -> int:
        speed = clamp_speed(speed)
        if self.mode == "discrete":
            return quantize_motor_speed(speed)

        self.reset_if_needed(speed)
        low, high, duty_high = effective_to_safe_pair(speed)
        if low == high or duty_high <= 0.0:
            return low
        if duty_high >= 1.0:
            return high

        if self.mode == "sigma":
            self.accumulator += duty_high
            if self.accumulator >= 1.0:
                self.accumulator -= 1.0
                return high
            return low

        if self.mode == "spread":
            high_ticks = round(duty_high * DITHER_SPREAD_TICKS)
            self.accumulator += high_ticks
            if self.accumulator >= DITHER_SPREAD_TICKS:
                self.accumulator -= DITHER_SPREAD_TICKS
                return high
            return low

        raise ValueError(f"Modo de dithering desconhecido: {self.mode}")


def get_motor_payload(port: int, speed: int) -> bytes:
    speed = quantize_motor_speed(speed)
    return bytes([port, 0x01, 0x01, speed & 0xFF])


def apply_direction(speed: int, invert: bool) -> int:
    speed = clamp_speed(speed)
    return -speed if invert else speed


def apply_power_budget(speed_a: int, speed_b: int) -> tuple[int, int]:
    total = abs(speed_a) + abs(speed_b)
    if total <= POWER_BUDGET:
        return speed_a, speed_b
    scale = POWER_BUDGET / total
    return int(speed_a * scale), int(speed_b * scale)


def smooth_speed(current: float, target: int) -> float:
    if target == 0:
        return 0.0
    next_value = current + SMOOTHING * (target - current)
    if abs(target) == MAX_POWER and abs(target - next_value) < 1.0:
        return float(target)
    return next_value


def rounded_speed(value: float) -> int:
    if abs(value) < 0.5:
        return 0
    return clamp_speed(round(value))


def set_motor_targets(speed_a: int, speed_b: int) -> None:
    with state.lock:
        state.speed_a = clamp_speed(speed_a)
        state.speed_b = clamp_speed(speed_b)


# ---------------------------------------------------------------------------
# BLE — comunicacao com o hub WeDo 2.0
# ---------------------------------------------------------------------------
async def write_motor(client: BleakClient, port: int, speed: int) -> None:
    await client.write_gatt_char(
        CHAR_WRITE,
        get_motor_payload(port, speed),
        response=False,
    )


async def stop_motors(client: BleakClient) -> None:
    await write_motor(client, PORT_A, 0)
    await asyncio.sleep(0.04)
    await write_motor(client, PORT_B, 0)


async def motor_self_test(client: BleakClient) -> None:
    print(
        "[BLE] Autoteste: motor A, depois motor B "
        f"({SELF_TEST_POWER}% por {SELF_TEST_SECONDS:.2f}s)."
    )
    await write_motor(client, PORT_A, SELF_TEST_POWER)
    await asyncio.sleep(SELF_TEST_SECONDS)
    await write_motor(client, PORT_A, 0)
    await asyncio.sleep(0.20)
    await write_motor(client, PORT_B, SELF_TEST_POWER)
    await asyncio.sleep(SELF_TEST_SECONDS)
    await write_motor(client, PORT_B, 0)


def handle_power_alert(kind: str, active: bool) -> None:
    now = time.monotonic()
    with state.lock:
        if kind == "low_voltage":
            state.low_voltage = active
        elif kind == "high_current":
            state.high_current = active
        if active:
            state.speed_a = 0
            state.speed_b = 0
            state.cooldown_until = max(state.cooldown_until, now + POWER_ALERT_COOLDOWN)
    if active:
        label = "baixa tensao" if kind == "low_voltage" else "corrente alta"
        print(f"[BLE] Alerta de {label}: parando motores por seguranca.")


def handle_low_voltage(_sender, data: bytearray) -> None:
    handle_power_alert("low_voltage", bool(data and data[0] != 0))


def handle_high_current(_sender, data: bytearray) -> None:
    handle_power_alert("high_current", bool(data and data[0] != 0))


async def connect_hub() -> BleakClient | None:
    print("[BLE] Conectando...")
    try:
        client = BleakClient(HUB_ADDRESS, timeout=10.0)
        await client.connect()
        print(f"[BLE] Conectado pelo MAC: {client.address}")
        return client
    except Exception as exc:
        print(f"[BLE] Falha ao conectar no MAC. Buscando pelo nome... {exc}")

    dev = await BleakScanner.find_device_by_name(HUB_NAME, timeout=10.0)
    if not dev:
        print("[BLE] Hub nao encontrado.")
        return None

    client = BleakClient(dev)
    await client.connect()
    print("[BLE] Conectado pelo scan.")
    return client


async def ble_loop() -> None:
    client = await connect_hub()
    if not client:
        state.running = False
        return

    state.connected = True
    last_sent_a = None
    last_sent_b = None
    notify_chars = []
    dual_start_until = 0.0
    dither_a = MotorDither(DITHER_MODE)
    dither_b = MotorDither(DITHER_MODE)

    print(f"[BLE] Dithering: modo={DITHER_MODE}, periodo={BLE_INTERVAL:.3f}s")

    try:
        if ENABLE_POWER_ALERTS:
            for char, handler, label in (
                (CHAR_LOW_VOLTAGE, handle_low_voltage, "baixa tensao"),
                (CHAR_HIGH_CURRENT, handle_high_current, "corrente alta"),
            ):
                try:
                    await client.start_notify(char, handler)
                    notify_chars.append(char)
                    print(f"[BLE] Monitorando alerta de {label}.")
                except Exception as exc:
                    print(f"[BLE] Nao foi possivel assinar {label}: {exc}")

        await stop_motors(client)
        last_sent_a = 0
        last_sent_b = 0

        if MOTOR_SELF_TEST:
            await motor_self_test(client)
            await stop_motors(client)
            last_sent_a = 0
            last_sent_b = 0

        while state.running:
            now = time.monotonic()
            with state.lock:
                target_a = state.speed_a
                target_b = state.speed_b
                in_cooldown = now < state.cooldown_until

            if in_cooldown:
                target_a = 0
                target_b = 0

            target_a = apply_direction(target_a, INVERT_A)
            target_b = apply_direction(target_b, INVERT_B)
            target_a, target_b = apply_power_budget(target_a, target_b)

            if target_a == 0 or target_b == 0:
                dual_start_until = 0.0
            elif last_sent_a == 0 and last_sent_b == 0 and now >= dual_start_until:
                dual_start_until = now + DUAL_START_STAGGER

            if now < dual_start_until and target_a != 0 and target_b != 0:
                target_b = 0

            command_a = dither_a.next_command(target_a)
            command_b = dither_b.next_command(target_b)

            if command_a != last_sent_a:
                await write_motor(client, PORT_A, command_a)
                last_sent_a = command_a
                if command_b != last_sent_b:
                    await asyncio.sleep(0.04)

            if command_b != last_sent_b:
                await write_motor(client, PORT_B, command_b)
                last_sent_b = command_b

            elapsed = time.monotonic() - now
            sleep_time = BLE_INTERVAL - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                await asyncio.sleep(0)

    except Exception:
        print("[BLE] Erro no loop BLE:")
        traceback.print_exc()
        state.running = False
    finally:
        print("[BLE] Parando motores e desconectando...")
        if client.is_connected:
            try:
                await stop_motors(client)
                for char in notify_chars:
                    await client.stop_notify(char)
            finally:
                await client.disconnect()
        state.connected = False


# ---------------------------------------------------------------------------
# Arcade mix — converte eixos analogicos em velocidade dos motores
# ---------------------------------------------------------------------------
def axis_to_power(value: float) -> int:
    if value == 0.0:
        return 0
    magnitude = abs(value)
    power = MIN_POWER + (MAX_POWER - MIN_POWER) * magnitude
    return int(power * (1 if value > 0 else -1))


def mix_arcade(throttle: float, steering: float) -> tuple[int, int]:
    left = max(-1.0, min(1.0, throttle + steering))
    right = max(-1.0, min(1.0, throttle - steering))
    return axis_to_power(left), axis_to_power(right)


# ---------------------------------------------------------------------------
# Evdev helpers — leitura de /dev/input/event*
# ---------------------------------------------------------------------------
@dataclass
class AbsInfo:
    minimum: int = -32768
    maximum: int = 32767
    flat: int = 0

    @property
    def center(self) -> int:
        return round((self.maximum + self.minimum) / 2.0)


def _ioc(direction: int, type_: int, nr: int, size: int) -> int:
    return (
        (direction << _IOC_DIRSHIFT)
        | (type_ << _IOC_TYPESHIFT)
        | (nr << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def eviocgabs(axis: int) -> int:
    return _ioc(_IOC_READ, ord("E"), 0x40 + axis, struct.calcsize("iiiiii"))


def read_abs_info(fd: int, axis: int) -> AbsInfo:
    data = bytearray(struct.calcsize("iiiiii"))
    try:
        fcntl.ioctl(fd, eviocgabs(axis), data, True)
        _value, minimum, maximum, _fuzz, flat, _resolution = struct.unpack("iiiiii", data)
        if minimum != maximum:
            return AbsInfo(minimum=minimum, maximum=maximum, flat=max(0, flat))
    except OSError:
        pass
    return AbsInfo()


def normalize_abs(raw_value: int, info: AbsInfo, dead_zone: float) -> float:
    """Normaliza um valor bruto de eixo absoluto para [-1.0, 1.0] com zona morta."""
    center = (info.maximum + info.minimum) / 2.0
    span = max(1.0, (info.maximum - info.minimum) / 2.0)
    value = max(-1.0, min(1.0, (raw_value - center) / span))
    flat = min(0.8, info.flat / span)
    dead_zone = max(dead_zone, flat)
    if abs(value) < dead_zone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    magnitude = (abs(value) - dead_zone) / (1.0 - dead_zone)
    return sign * max(0.0, min(1.0, magnitude))


# AbsInfo padrao para joydev (js*): eixos ja vem normalizados em -32767..32767
_JOYDEV_ABS_INFO = AbsInfo(minimum=-32767, maximum=32767, flat=0)


def normalize_axis(raw_value: int, dead_zone: float) -> float:
    """Normaliza eixo joydev (wrapper sobre normalize_abs com range padrao)."""
    return normalize_abs(raw_value, _JOYDEV_ABS_INFO, dead_zone)


def event_device_names() -> dict[str, str]:
    names: dict[str, str] = {}
    current_name = ""
    handlers: list[str] = []
    try:
        with open("/proc/bus/input/devices", "r", encoding="utf-8") as devices:
            for line in devices:
                line = line.rstrip()
                if not line:
                    for handler in handlers:
                        if handler.startswith("event"):
                            names[f"/dev/input/{handler}"] = current_name
                    current_name = ""
                    handlers = []
                    continue
                if line.startswith("N: Name="):
                    current_name = line.split("=", 1)[1].strip().strip('"')
                elif line.startswith("H: Handlers="):
                    handlers = line.split("=", 1)[1].split()
    except OSError:
        pass
    return names


def find_event_device() -> str | None:
    names = event_device_names()
    # Ordena numericamente: event8 antes de event10
    candidates = sorted(
        glob.glob("/dev/input/event*"),
        key=lambda p: int(p.rsplit("event", 1)[-1]) if p.rsplit("event", 1)[-1].isdigit() else 999,
    )
    # Palavras-chave que identificam controles/gamepads
    keywords = ("gamepad", "joystick", "controller", "xbox", "x-box", "wireless gamepad")
    # Termos que indicam que NAO e um controle
    exclude = ("touchpad", "mouse", "keyboard", "audio", "hdmi", "speaker", "lid", "power", "ideapad", "video")

    for path in candidates:
        name = names.get(path, "").lower()
        if any(kw in name for kw in keywords):
            if not any(ex in name for ex in exclude):
                return path

    # Fallback: dispositivo com eixos absolutos (EV_ABS) que nao e mouse/touchpad
    EV_ABS_BIT = 1 << 0x03
    abs_candidates = []
    for path in candidates:
        name = names.get(path, "").lower()
        if any(ex in name for ex in exclude):
            continue
        try:
            with open(path, "rb") as f:
                buf = bytearray(4)
                fcntl.ioctl(f.fileno(), 0x80044565, buf, True)  # EVIOCGBIT(0, 4)
                bits = int.from_bytes(buf, "little")
                if bits & EV_ABS_BIT:
                    abs_candidates.append(path)
        except OSError:
            pass
    if abs_candidates:
        return abs_candidates[-1]

    return candidates[0] if len(candidates) == 1 else None


def print_devices() -> None:
    names = event_device_names()
    for path in sorted(glob.glob("/dev/input/event*")):
        print(f"{path}\t{names.get(path, '(sem nome)')}")


# ---------------------------------------------------------------------------
# Status de impressao (unificado)
# ---------------------------------------------------------------------------
def print_status(speed_a: int, speed_b: int, prefix: str = "JOY") -> None:
    with state.lock:
        low_voltage = state.low_voltage
        high_current = state.high_current
        in_cooldown = time.monotonic() < state.cooldown_until
        connected = state.connected

    if low_voltage:
        status = "BAIXA TENSAO"
    elif high_current or in_cooldown:
        status = "PROTECAO DE CORRENTE"
    elif connected:
        status = "BLE CONECTADO"
    else:
        status = "AGUARDANDO BLE"

    print(f"\r[{prefix}] {status} | Motor A: {speed_a:>4}% | Motor B: {speed_b:>4}%  [Q=sair]", end="")


# ---------------------------------------------------------------------------
# Calculo de alvos para modo arcade (evdev)
# ---------------------------------------------------------------------------
def calc_targets(args: argparse.Namespace, axes: dict[int, int], axis_info: dict[int, AbsInfo]) -> tuple[int, int]:
    throttle = -normalize_abs(
        axes.get(args.throttle_axis, axis_info[args.throttle_axis].center),
        axis_info[args.throttle_axis],
        args.dead_zone,
    )
    steering = normalize_abs(
        axes.get(args.steering_axis, axis_info[args.steering_axis].center),
        axis_info[args.steering_axis],
        args.dead_zone,
    )
    return mix_arcade(
        throttle * args.speed_scale,
        steering * args.turn_scale,
    )


# ---------------------------------------------------------------------------
# Loop de entrada — evdev (/dev/input/event*)
# ---------------------------------------------------------------------------
def event_loop(args: argparse.Namespace) -> None:
    device = args.device
    if device == "auto":
        device = find_event_device()

    if not device:
        print("[EV] Nao consegui escolher um controle automaticamente.")
        print("[EV] Rode: python joy_event.py --list-devices")
        state.running = False
        return

    if not os.path.exists(device):
        print(f"[EV] Dispositivo nao encontrado: {device}")
        state.running = False
        return

    axes: dict[int, int] = {}
    buttons: dict[int, int] = {}
    smooth_a = 0.0
    smooth_b = 0.0
    last_event = time.monotonic()
    last_print = 0.0

    print(f"[EV] Lendo controle em {device}")
    print("[EV] Modo arcade (invertido) - analogico esquerdo:")
    print("[EV]   Y para cima/baixo  -> ambos motores tras/frente")
    print("[EV]   X para direita     -> Motor B frente, Motor A tras (vira esquerda)")
    print("[EV]   X para esquerda    -> Motor A frente, Motor B tras (vira direita)")
    print("[EV]   Tecla Q            -> encerra o programa")

    axis_codes = {args.throttle_axis, args.steering_axis}
    last_debug_print: dict[int, int] = {}

    with open(device, "rb", buffering=0) as controller:
        fd = controller.fileno()
        axis_info = {axis: read_abs_info(fd, axis) for axis in axis_codes}
        for axis, info in axis_info.items():
            axes[axis] = info.center

        if args.debug:
            print("[DEBUG] Modo de mapeamento ativo. Mova os analogicos para ver os codigos dos eixos.")
            print("[DEBUG] Formato: EV_ABS codigo=<N> valor=<V>  (use --throttle-axis e --steering-axis para configurar)")

        while state.running:
            readable, _, _ = select.select([controller], [], [], 0.05)
            now = time.monotonic()

            if readable:
                event = controller.read(INPUT_EVENT_SIZE)
                if len(event) != INPUT_EVENT_SIZE:
                    print("\n[EV] Controle desconectado.")
                    break

                _sec, _usec, event_type, code, value = struct.unpack(_INPUT_EVENT_FMT, event)
                last_event = now

                if event_type == EV_ABS:
                    if code not in axis_info:
                        axis_info[code] = read_abs_info(fd, code)
                        axes[code] = axis_info[code].center
                    axes[code] = value
                    if args.debug and last_debug_print.get(code) != value:
                        last_debug_print[code] = value
                        norm = normalize_abs(value, axis_info[code], args.dead_zone)
                        print(f"\r[DEBUG] EV_ABS codigo={code:>2}  raw={value:>6}  norm={norm:+.3f}   ", flush=True)
                elif event_type == EV_KEY:
                    buttons[code] = value
                    if args.debug:
                        print(f"\r[DEBUG] EV_KEY  codigo={code:>3}  valor={value}   ", flush=True)

            timed_out = now - last_event > args.timeout
            stop_pressed = buttons.get(args.stop_button, 0) == 1
            deadman_missing = (
                args.deadman_button is not None
                and buttons.get(args.deadman_button, 0) != 1
            )

            if timed_out or stop_pressed or deadman_missing:
                target_a = 0
                target_b = 0
            else:
                target_a, target_b = calc_targets(args, axes, axis_info)

            smooth_a = smooth_speed(smooth_a, target_a)
            smooth_b = smooth_speed(smooth_b, target_b)
            speed_a = rounded_speed(smooth_a)
            speed_b = rounded_speed(smooth_b)
            set_motor_targets(speed_a, speed_b)

            if now - last_print >= 0.15:
                print_status(speed_a, speed_b, prefix="EV")
                last_print = now

    set_motor_targets(0, 0)
    state.running = False
    print("\n[EV] Encerrando; motores em zero.")


# ---------------------------------------------------------------------------
# Loop de entrada — joydev (/dev/input/js*)
# ---------------------------------------------------------------------------
def joystick_loop(args: argparse.Namespace) -> None:
    if not os.path.exists(args.device):
        print(f"[JOY] Dispositivo nao encontrado: {args.device}")
        print("[JOY] Pareie o controle Bluetooth e verifique com: ls /dev/input/js*")
        state.running = False
        return

    axes: dict[int, int] = {}
    buttons: dict[int, int] = {}
    smooth_a = 0.0
    smooth_b = 0.0
    last_event = time.monotonic()
    last_print = 0.0

    print(f"[JOY] Lendo joystick em {args.device}")
    print("[JOY] Controle (invertido): eixo Y esquerdo freia/acelera, eixo X esquerdo vira ao contrario, botao 0 para.")

    with open(args.device, "rb", buffering=0) as joystick:
        while state.running:
            readable, _, _ = select.select([joystick], [], [], 0.05)
            now = time.monotonic()

            if readable:
                event = joystick.read(JS_EVENT_SIZE)
                if len(event) != JS_EVENT_SIZE:
                    print("\n[JOY] Joystick desconectado.")
                    break

                _event_time, value, event_type, number = struct.unpack("IhBB", event)
                event_type &= ~JS_EVENT_INIT
                last_event = now

                if event_type == JS_EVENT_AXIS:
                    axes[number] = value
                elif event_type == JS_EVENT_BUTTON:
                    buttons[number] = value

            timed_out = now - last_event > args.timeout
            stop_pressed = buttons.get(args.stop_button, 0) == 1
            deadman_missing = (
                args.deadman_button is not None
                and buttons.get(args.deadman_button, 0) != 1
            )

            if timed_out or stop_pressed or deadman_missing:
                target_a = 0
                target_b = 0
            else:
                throttle = normalize_axis(axes.get(args.throttle_axis, 0), args.dead_zone)
                steering = -normalize_axis(axes.get(args.steering_axis, 0), args.dead_zone)
                target_a, target_b = mix_arcade(
                    throttle * args.speed_scale,
                    steering * args.turn_scale,
                )

            smooth_a = smooth_speed(smooth_a, target_a)
            smooth_b = smooth_speed(smooth_b, target_b)
            speed_a = rounded_speed(smooth_a)
            speed_b = rounded_speed(smooth_b)
            set_motor_targets(speed_a, speed_b)

            if now - last_print >= 0.15:
                print_status(speed_a, speed_b, prefix="JOY")
                last_print = now

    set_motor_targets(0, 0)
    state.running = False
    print("\n[JOY] Encerrando; motores em zero.")


# ---------------------------------------------------------------------------
# Keyboard watcher — tecla Q encerra o programa
# ---------------------------------------------------------------------------
def keyboard_watcher() -> None:
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while state.running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                ch = sys.stdin.read(1)
                if ch.lower() == "q":
                    print("\n[SYS] Tecla Q pressionada. Encerrando...")
                    state.running = False
                    break
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ---------------------------------------------------------------------------
# Deteccao automatica do modo de entrada
# ---------------------------------------------------------------------------
def detect_input_mode(device: str) -> str:
    """Retorna 'joydev' se o dispositivo e /dev/input/js*, senao 'evdev'."""
    if device.startswith("/dev/input/js"):
        return "joydev"
    if device.startswith("/dev/input/event"):
        return "evdev"
    # Se e 'auto', tenta js0 primeiro, depois event*
    if device == "auto":
        if os.path.exists("/dev/input/js0"):
            return "joydev"
        return "evdev"
    return "evdev"


# ---------------------------------------------------------------------------
# Argumentos de linha de comando (unificado)
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Controla o SmartCar com joystick Bluetooth (joydev ou evdev)."
    )
    parser.add_argument(
        "--device", default="auto",
        help="dispositivo de entrada: /dev/input/js0, /dev/input/event*, ou 'auto'",
    )
    parser.add_argument("--list-devices", action="store_true", help="lista dispositivos event")
    parser.add_argument(
        "--throttle-axis", type=int, default=None,
        help="eixo do acelerador (padrao: 1 para joydev, ABS_Y para evdev)",
    )
    parser.add_argument(
        "--steering-axis", type=int, default=None,
        help="eixo da direcao (padrao: 0 para joydev, ABS_X para evdev)",
    )
    parser.add_argument(
        "--stop-button", type=int, default=None,
        help="botao que zera os motores (padrao: 0 para joydev, BTN_SOUTH para evdev)",
    )
    parser.add_argument(
        "--deadman-button", type=int, default=None,
        help="se definido, o carrinho so anda enquanto este botao estiver pressionado",
    )
    parser.add_argument("--speed-scale", type=float, default=1.0, help="escala do acelerador (0.0 a 1.0)")
    parser.add_argument("--turn-scale", type=float, default=0.85, help="escala da direcao (0.0 a 1.0)")
    parser.add_argument("--dead-zone", type=float, default=DEAD_ZONE, help="zona morta dos analogicos")
    parser.add_argument("--timeout", type=float, default=0.75, help="segundos sem eventos ate parar")
    parser.add_argument(
        "--dither", choices=("sigma", "spread", "discrete"), default=DITHER_MODE,
        help="modo para sintetizar velocidades intermediarias sem enviar 56..95",
    )
    parser.add_argument(
        "--period", type=float, default=BLE_INTERVAL,
        help="periodo BLE em segundos para o dithering",
    )
    parser.add_argument("--no-self-test", action="store_true", help="desativa o autoteste inicial dos motores")
    parser.add_argument(
        "--debug", action="store_true",
        help="imprime codigos e valores brutos de todos os eixos (use para mapear o controle)",
    )
    parser.add_argument("--invert-a", action="store_true", help="inverte a direcao do motor A")
    parser.add_argument("--invert-b", action="store_true", help="inverte a direcao do motor B")
    parser.add_argument(
        "--no-stagger", action="store_true",
        help="desativa o atraso de partida dual (elimina o giro inicial ao arrancar)",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    args = parse_args()

    if args.list_devices:
        print_devices()
        raise SystemExit(0)

    # Configuracoes globais
    DITHER_MODE = args.dither
    BLE_INTERVAL = max(0.04, args.period)
    MOTOR_SELF_TEST = not args.no_self_test
    args.dead_zone = max(0.0, min(0.8, args.dead_zone))

    if args.invert_a:
        INVERT_A = not INVERT_A
    if args.invert_b:
        INVERT_B = not INVERT_B
    if args.no_stagger:
        DUAL_START_STAGGER = 0.0

    # Detecta modo de entrada e ajusta defaults dos eixos/botoes
    mode = detect_input_mode(args.device)

    if mode == "joydev":
        if args.device == "auto":
            args.device = "/dev/input/js0"
        if args.throttle_axis is None:
            args.throttle_axis = 1
        if args.steering_axis is None:
            args.steering_axis = 0
        if args.stop_button is None:
            args.stop_button = 0
    else:  # evdev
        if args.throttle_axis is None:
            args.throttle_axis = ABS_Y
        if args.steering_axis is None:
            args.steering_axis = ABS_X
        if args.stop_button is None:
            args.stop_button = BTN_SOUTH

    # Thread: teclado (tecla Q)
    kb_thread = threading.Thread(target=keyboard_watcher, daemon=True)
    kb_thread.start()

    # Thread: BLE
    ble_thread = threading.Thread(target=lambda: asyncio.run(ble_loop()), daemon=True)
    ble_thread.start()

    try:
        if mode == "joydev":
            joystick_loop(args)
        else:
            event_loop(args)
    except KeyboardInterrupt:
        set_motor_targets(0, 0)
        state.running = False
        print("\n[SYS] Interrompido pelo usuario.")
    finally:
        state.running = False
        ble_thread.join()
        print("[SYS] Fim.")
