#!/usr/bin/env python3
"""
SmartCar - controlo continuo com protecao de corrente para LEGO WeDo 2.0.
"""

import asyncio
import argparse
import os
import threading
import time
import traceback
import urllib.request

import cv2
import mediapipe as mp
from bleak import BleakClient, BleakScanner

# BLE / WeDo 2.0
HUB_ADDRESS = "74:8B:34:94:63:3B"
HUB_NAME = "M_SmartCar_2/0"

CHAR_WRITE = "00001565-1212-efde-1523-785feabcd123"
CHAR_LOW_VOLTAGE = "00001528-1212-efde-1523-785feabcd123"
CHAR_HIGH_CURRENT = "00001529-1212-efde-1523-785feabcd123"

PORT_A = 0x01
PORT_B = 0x02

# Em carros com motores montados espelhados, um lado normalmente precisa
# inverter o sinal para os dois lados empurrarem o carro na mesma direcao.
INVERT_A = False
INVERT_B = True

# Motor e visao
DEAD_ZONE = 0.10
MIN_POWER = 55
MAX_POWER = 100
SAFE_MOTOR_LEVELS = (0, 55, 100)
FULL_SPEED_EDGE = 0.08
HAND_CONFIDENCE_MIN = 0.65
HAND_LOST_TIMEOUT = 0.45

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
WINDOW_NAME = "SmartCar - Controlo Perfeito"

# Download do modelo
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
)
MODEL_PATH = "pose_landmarker_lite.task"

if not os.path.isfile(MODEL_PATH):
    print("[MODEL] Baixando modelo...")
    urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


class SharedState:
    def __init__(self):
        self.speed_a = 0
        self.speed_b = 0
        self.connected = False
        self.running = True
        self.low_voltage = False
        self.high_current = False
        self.cooldown_until = 0.0
        self.lock = threading.Lock()


state = SharedState()


def clamp_speed(speed: int) -> int:
    return max(-100, min(100, int(speed)))


def quantize_motor_speed(speed: int) -> int:
    """Usa apenas os niveis que o hub/motores aceitaram no teste local."""
    speed = clamp_speed(speed)
    if speed == 0:
        return SAFE_MOTOR_LEVELS[0]

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
    # WeDo 2.0 motor na caracteristica 1565:
    # [porta, comando motor, tamanho, percentual int8].
    # Stop = 0x00; frente = 0x01..0x64; reverso = 0xff..0x9c.
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


def landmark_score(landmark) -> float:
    visibility = getattr(landmark, "visibility", 1.0)
    presence = getattr(landmark, "presence", 1.0)
    if visibility is None:
        visibility = 1.0
    if presence is None:
        presence = 1.0
    return min(float(visibility), float(presence))


def is_valid_wrist(landmark) -> bool:
    if landmark_score(landmark) < HAND_CONFIDENCE_MIN:
        return False
    return 0.0 <= float(landmark.x) <= 1.0 and 0.0 <= float(landmark.y) <= 1.0


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


class HandHold:
    def __init__(self) -> None:
        self.speed = 0
        self.last_seen = 0.0

    def update(self, detected: bool, speed: int, now: float) -> int:
        if detected:
            self.speed = speed
            self.last_seen = now
            return speed

        if now - self.last_seen <= HAND_LOST_TIMEOUT:
            return self.speed

        self.speed = 0
        return 0


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
            state.cooldown_until = max(
                state.cooldown_until,
                now + POWER_ALERT_COOLDOWN,
            )

    if active:
        label = "baixa tensao" if kind == "low_voltage" else "corrente alta"
        print(f"[BLE] Alerta de {label}: parando motores por seguranca.")


def handle_low_voltage(_sender, data: bytearray) -> None:
    handle_power_alert("low_voltage", bool(data and data[0] != 0))


def handle_high_current(_sender, data: bytearray) -> None:
    handle_power_alert("high_current", bool(data and data[0] != 0))


def clear_power_alerts() -> None:
    with state.lock:
        state.low_voltage = False
        state.high_current = False


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


def run_ble_thread() -> None:
    asyncio.run(ble_loop())


def calc_speed(y_norm: float) -> int:
    y_norm = max(0.0, min(1.0, float(y_norm)))
    if y_norm <= FULL_SPEED_EDGE:
        return MAX_POWER
    if y_norm >= 1.0 - FULL_SPEED_EDGE:
        return -MAX_POWER

    dev = 0.5 - y_norm

    if abs(dev) < DEAD_ZONE:
        return 0

    magnitude = (abs(dev) - DEAD_ZONE) / (0.5 - DEAD_ZONE)
    magnitude = max(0.0, min(1.0, magnitude))
    power = MIN_POWER + (MAX_POWER - MIN_POWER) * magnitude
    sign = 1 if dev > 0 else -1
    return int(power * sign)


def vision_loop() -> None:
    options = mp.tasks.vision.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 720)

    smooth_a = 0.0
    smooth_b = 0.0
    hold_a = HandHold()
    hold_b = HandHold()

    print("[CAM] Iniciando captura...")
    with mp.tasks.vision.PoseLandmarker.create_from_options(options) as landmarker:
        start_ms = time.time() * 1000

        while state.running:
            ret, frame = cap.read()
            if not ret:
                print("[CAM] Falha ao ler frame da camera; encerrando.")
                break

            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = int(time.time() * 1000 - start_ms)

            results = landmarker.detect_for_video(mp_image, ts)
            now = time.monotonic()
            target_a = 0
            target_b = 0
            seen_a = False
            seen_b = False

            mid_y = h // 2
            dz_px = int(DEAD_ZONE * h)
            cv2.line(frame, (0, mid_y), (w, mid_y), (200, 200, 200), 2)
            cv2.line(frame, (0, mid_y - dz_px), (w, mid_y - dz_px), (0, 255, 0), 1)
            cv2.line(frame, (0, mid_y + dz_px), (w, mid_y + dz_px), (0, 255, 0), 1)

            if results.pose_landmarks:
                lm = results.pose_landmarks[0]

                rw = lm[15]
                lw = lm[16]

                if is_valid_wrist(rw):
                    target_a = calc_speed(rw.y)
                    seen_a = True
                    cv2.circle(frame, (int(rw.x * w), int(rw.y * h)), 12, (0, 200, 255), -1)

                if is_valid_wrist(lw):
                    target_b = calc_speed(lw.y)
                    seen_b = True
                    cv2.circle(frame, (int(lw.x * w), int(lw.y * h)), 12, (255, 128, 0), -1)

            target_a = hold_a.update(seen_a, target_a, now)
            target_b = hold_b.update(seen_b, target_b, now)

            smooth_a = smooth_speed(smooth_a, target_a)
            smooth_b = smooth_speed(smooth_b, target_b)

            with state.lock:
                state.speed_a = rounded_speed(smooth_a)
                state.speed_b = rounded_speed(smooth_b)
                low_voltage = state.low_voltage
                high_current = state.high_current
                in_cooldown = time.monotonic() < state.cooldown_until

            status = "BLE CONECTADO" if state.connected else "AGUARDANDO BLE..."
            color = (0, 255, 0) if state.connected else (0, 0, 255)
            if low_voltage:
                status = "BAIXA TENSAO"
                color = (0, 165, 255)
            elif high_current or in_cooldown:
                status = "PROTECAO DE CORRENTE"
                color = (0, 165, 255)

            cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(frame, f"Motor A: {state.speed_a}%", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(frame, f"Motor B: {state.speed_b}%", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    state.running = False
    print("[CAM] Encerrando captura; sinalizando BLE.")
    cap.release()
    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dither",
        choices=("sigma", "spread", "discrete"),
        default=DITHER_MODE,
        help="modo para sintetizar velocidades intermediarias sem enviar 56..95",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=BLE_INTERVAL,
        help="periodo BLE em segundos para o dithering",
    )
    parser.add_argument(
        "--no-self-test",
        action="store_true",
        help="desativa o autoteste inicial dos motores",
    )
    parser.add_argument(
        "--hand-confidence",
        type=float,
        default=HAND_CONFIDENCE_MIN,
        help="confianca minima do pulso para aceitar comando",
    )
    parser.add_argument(
        "--full-speed-edge",
        type=float,
        default=FULL_SPEED_EDGE,
        help="faixa perto do topo/fundo da tela que satura em 100",
    )
    parser.add_argument(
        "--hand-timeout",
        type=float,
        default=HAND_LOST_TIMEOUT,
        help="segundos para manter ultimo comando valido quando o pulso some",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    DITHER_MODE = args.dither
    BLE_INTERVAL = max(0.04, args.period)
    MOTOR_SELF_TEST = not args.no_self_test
    HAND_CONFIDENCE_MIN = max(0.0, min(1.0, args.hand_confidence))
    FULL_SPEED_EDGE = max(0.0, min(0.25, args.full_speed_edge))
    HAND_LOST_TIMEOUT = max(0.0, args.hand_timeout)

    ble_thread = threading.Thread(target=run_ble_thread, daemon=True)
    ble_thread.start()

    try:
        vision_loop()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        ble_thread.join()
        print("[SYS] Fim.")
