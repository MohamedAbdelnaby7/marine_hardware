import threading
import time
import logging
import sys
from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger("fake_vehicles")

VEHICLES = [
    {"name": "ROV1", "port": 14550, "type": mavutil.mavlink.MAV_TYPE_SUBMARINE},
    {"name": "ROV2", "port": 14551, "type": mavutil.mavlink.MAV_TYPE_SUBMARINE},
    {"name": "Boat", "port": 14552, "type": mavutil.mavlink.MAV_TYPE_SURFACE_BOAT},
]

def run_vehicle(name, port, mav_type):
    log.info("[%s] Listening on UDP port %d", name, port)
    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}",
        source_system=1,
        source_component=1
    )
    armed = True

    def send_heartbeats():
        while True:
            try:
                conn.mav.heartbeat_send(
                    mav_type,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE if armed
                    else mavutil.mavlink.MAV_STATE_STANDBY
                )
            except Exception:
                pass
            time.sleep(1)

    threading.Thread(target=send_heartbeats, daemon=True).start()
    log.info("[%s] Armed and running", name)

    while True:
        try:
            msg = conn.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_LONG":
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.param1 == 0:
                        armed = False
                        log.critical("[%s] 🔴 DISARM RECEIVED — vehicle stopped", name)
                    else:
                        armed = True
                        log.info("[%s] ✅ ARM RECEIVED", name)
                    conn.mav.command_ack_send(
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        0, 0, 0, 0
                    )
        except Exception:
            pass

for v in VEHICLES:
    threading.Thread(
        target=run_vehicle,
        args=(v["name"], v["port"], v["type"]),
        daemon=True
    ).start()

log.info("=== FAKE VEHICLES RUNNING — ROV1:14550  ROV2:14551  Boat:14552 ===")
while True:
    time.sleep(1)