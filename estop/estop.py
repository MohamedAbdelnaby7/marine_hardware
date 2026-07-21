import time
import threading
import logging
import yaml
import sys

from gpiozero import Button, LED
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger("estop")

# ── Load config ────────────────────────────────────────────────────────────────
with open("/app/config.yml") as f:
    config = yaml.safe_load(f)

VEHICLES    = config["vehicles"]
BUTTON_PIN  = config["gpio"]["button_pin"]
GREEN_PIN   = config["gpio"]["green_led_pin"]
RED_PIN     = config["gpio"]["red_led_pin"]
DISARM_MS   = config["disarm_interval_ms"] / 1000.0
HB_INTERVAL = config["heartbeat_interval_s"]

# ── Global state ───────────────────────────────────────────────────────────────
lockout_active = threading.Event()
connections    = {}   # name → mavutil connection

# ── GPIO setup (Pi 5 compatible) ───────────────────────────────────────────────
button    = Button(BUTTON_PIN, pull_up=True, bounce_time=0.3)
green_led = LED(GREEN_PIN)
red_led   = LED(RED_PIN)

# ── MAVLink helpers ────────────────────────────────────────────────────────────
def connect_vehicle(vehicle: dict):
    name     = vehicle["name"]
    conn_str = vehicle["connection"]
    log.info("Connecting to %s at %s ...", name, conn_str)
    try:
        conn = mavutil.mavlink_connection(conn_str, autoreconnect=True)
        # Send one heartbeat first so UDP vehicle knows our address
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        hb = conn.wait_heartbeat(timeout=10)
        if hb is None:
            log.warning("No heartbeat from %s after 10s — vehicle unreachable", name)
            return None
        log.info("Heartbeat received from %s (sys=%d, comp=%d)",
                 name, conn.target_system, conn.target_component)
        return conn
    except Exception as e:
        log.error("Could not connect to %s: %s", name, e)
        return None


def send_disarm(conn, name: str, force: bool = True):
    """
    Send MAV_CMD_COMPONENT_ARM_DISARM with param1=0 (disarm).
    param2=21196 is the ArduPilot force-disarm magic number —
    bypasses safety checks so the vehicle disarms even mid-operation.
    """
    try:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                          # confirmation
            0,                          # param1: 0 = disarm
            21196 if force else 0,      # param2: force disarm
            0, 0, 0, 0, 0
        )
        log.debug("Disarm sent to %s", name)
    except Exception as e:
        log.warning("Failed to send disarm to %s: %s", name, e)


def send_heartbeat(conn):
    """MAVLink connections drop without regular heartbeats from the GCS."""
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )


# ── Button callback ────────────────────────────────────────────────────────────
def on_button_press():
    if lockout_active.is_set():
        log.warning("E-STOP already active. System is in lockout.")
        return

    log.critical("🔴 E-STOP ACTIVATED")
    lockout_active.set()

    # Immediate disarm — hit all vehicles right now before lockout loop takes over
    for name, conn in connections.items():
        if conn:
            send_disarm(conn, name, force=True)
            log.info("  → Disarmed %s", name)

    # LEDs: green off, red on
    green_led.off()
    red_led.on()

    log.critical("System is now in LOCKOUT. Restart the container to clear.")


# ── Lockout enforcement loop ───────────────────────────────────────────────────
def lockout_loop():
    """
    Once lockout is active, continuously re-send disarm to every vehicle
    on a fixed interval. This prevents any operator from re-arming a vehicle
    while the physical e-stop button is still latched.
    Only a deliberate container restart can exit this loop.
    """
    while True:
        if lockout_active.is_set():
            for name, conn in connections.items():
                if conn:
                    send_disarm(conn, name, force=True)
        time.sleep(DISARM_MS)


# ── Heartbeat loop ─────────────────────────────────────────────────────────────
def heartbeat_loop():
    """
    MAVLink requires the GCS to send a heartbeat at least once per second
    or the vehicle considers the connection lost. We keep this running
    even during lockout so the disarm commands keep being acknowledged.
    """
    while True:
        for name, conn in connections.items():
            if conn:
                try:
                    send_heartbeat(conn)
                except Exception as e:
                    log.warning("Heartbeat failed for %s: %s", name, e)
        time.sleep(HB_INTERVAL)


# ── Startup blink ──────────────────────────────────────────────────────────────
def startup_blink():
    """Blink green LED while connecting to vehicles so you know the Pi is alive."""
    while not all(connections):
        green_led.on()
        time.sleep(0.2)
        green_led.off()
        time.sleep(0.2)


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    log.info("=== E-STOP SYSTEM STARTING ===")

    # Blink green while we connect
    blink_thread = threading.Thread(target=startup_blink, daemon=True)
    blink_thread.start()

    # Connect to all vehicles
    for vehicle in VEHICLES:
        conn = connect_vehicle(vehicle)
        connections[vehicle["name"]] = conn

    connected = sum(1 for c in connections.values() if c is not None)
    log.info("Connected to %d / %d vehicles", connected, len(VEHICLES))

    if connected == 0:
        log.error("No vehicles reachable. Check network and IPs in config.yml")
        red_led.on()
        sys.exit(1)

    if connected < len(VEHICLES):
        log.warning(
            "Only %d of %d vehicles reachable — continuing, but check your network.",
            connected, len(VEHICLES)
        )

    # Solid green = system is live and watching
    green_led.on()
    red_led.off()
    log.info("E-stop armed. Green LED ON. Watching for button press...")

    # Register button callback
    button.when_pressed = on_button_press

    # Start background threads
    threading.Thread(target=heartbeat_loop, daemon=True).start()
    threading.Thread(target=lockout_loop,   daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        green_led.off()
        red_led.off()


if __name__ == "__main__":
    main()