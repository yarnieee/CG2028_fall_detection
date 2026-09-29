"""Fall Detector Base Station web dashboard and sensor socket bridge.

The project is intentionally small, but the server owns the complete data flow:
sensor readings arrive over TCP, are saved to ``data/data.csv``, and are exposed
to the dashboard through Flask and Socket.IO.
"""

from dataclasses import asdict, dataclass, fields
from datetime import datetime
import html
import json
import os
from pathlib import Path
import re
import socket
import threading
import time
from typing import Any

import requests
from flask import Flask, jsonify, redirect, render_template, request, url_for
from flask_socketio import SocketIO


#################################### PATHS AND CONSTANTS ################################

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
STATIC_DIR = BASE_DIR / "static"
GRAPH_DIR = STATIC_DIR / "graphs"
DATA_DIR.mkdir(parents=True, exist_ok=True)
GRAPH_DIR.mkdir(parents=True, exist_ok=True)

SOCKET_IP = "10.249.88.87"
SOCKET_PORT = 2028
FLASK_IP = "127.0.0.1"
FLASK_PORT = 5000

FILE_PATH = DATA_DIR / "data.csv"
PARTICULARS_PATH = DATA_DIR / "particulars.json"
EMERGENCY_CONTACTS_PATH = DATA_DIR / "emergency_contacts.json"

# Credentials are read from the environment instead of being committed to the
# project. Telegram notifications are simply skipped when they are not set.
BOT_TOKEN = "8891411229:AAH3NPM-Qge5w3fhrQbOCcBOrg0ayQPkqAk"
CHAT_ID = "6345748326"

file_lock = threading.Lock()
profile_lock = threading.Lock()
last_fall_time = ""
last_sensor_state: int | None = None
alert_last_sent_at: dict[int, float] = {3: 0.0, 4: 0.0}
ALERT_COOLDOWN_SECONDS = 5 * 60


#################################### DATA CLASSES #######################################


@dataclass
class EmergencyContact:
    name: str = ""
    contact: str = ""
    relationship: str = ""


@dataclass
class PersonalInfo:
    name: str = ""
    DOB: str = ""
    blood_type: str = ""
    address: str = ""
    hospital: str = ""
    conditions: str = ""
    medications: str = ""
    allergies: str = ""


#################################### JSON STORAGE ######################################


def _read_json(path: Path, default: Any) -> Any:
    """Read a JSON file, returning the supplied default for a first launch."""
    try:
        if path.stat().st_size == 0:
            return default
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return default


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    with temporary_path.open("w", encoding="utf-8") as file:
        json.dump(value, file, indent=2)
        file.write("\n")
    temporary_path.replace(path)


def _personal_info_from_dict(value: Any) -> PersonalInfo:
    if not isinstance(value, dict):
        return PersonalInfo()
    valid_names = {field.name for field in fields(PersonalInfo)}
    cleaned = {name: str(value.get(name, "") or "").strip() for name in valid_names}
    return PersonalInfo(**cleaned)


def _contacts_from_json(value: Any) -> list[EmergencyContact]:
    if isinstance(value, dict):
        value = [value]
    if not isinstance(value, list):
        value = []

    contacts: list[EmergencyContact] = []
    for item in value:
        if not isinstance(item, dict):
            continue
        contacts.append(
            EmergencyContact(
                name=str(item.get("name", "") or "").strip(),
                contact=str(item.get("contact", "") or "").strip(),
                relationship=str(item.get("relationship", "") or "").strip(),
            )
        )
    return contacts or [EmergencyContact()]


personal_info = PersonalInfo()
emergency_contacts: list[EmergencyContact] = [EmergencyContact()]
# Kept as an alias for small scripts that imported the original starter variable.
emergency_contact = emergency_contacts[0]


def init_info() -> None:
    """Initialise and load the personal profile and emergency contact files."""
    global personal_info, emergency_contacts, emergency_contact

    with profile_lock:
        particulars = _read_json(PARTICULARS_PATH, {})
        contacts = _read_json(EMERGENCY_CONTACTS_PATH, [])
        personal_info = _personal_info_from_dict(particulars)
        emergency_contacts = _contacts_from_json(contacts)
        emergency_contact = emergency_contacts[0]

        # Empty starter files become valid JSON on first launch.
        if not PARTICULARS_PATH.exists() or PARTICULARS_PATH.stat().st_size == 0:
            _write_json(PARTICULARS_PATH, asdict(personal_info))
        if not EMERGENCY_CONTACTS_PATH.exists() or EMERGENCY_CONTACTS_PATH.stat().st_size == 0:
            _write_json(EMERGENCY_CONTACTS_PATH, [asdict(contact) for contact in emergency_contacts])


init_info()


#################################### SENSOR DATA #######################################


def write_to_file(data: str) -> None:
    """Append raw sensor data safely, preserving one reading per line."""
    if not data:
        return
    with file_lock:
        with FILE_PATH.open("a", encoding="utf-8") as file:
            for line in str(data).splitlines():
                if line.strip():
                    file.write(line.strip() + "\n")


def read_from_file() -> str:
    with file_lock:
        try:
            return FILE_PATH.read_text(encoding="utf-8")
        except FileNotFoundError:
            return ""


def parse_sensor_line(line: str) -> dict[str, Any] | None:
    """Parse ``timestamp, state, acceleration, gyroscope, sound`` readings."""
    clean_line = line.split("//", 1)[0].strip()
    if not clean_line:
        return None
    values = [value.strip() for value in clean_line.split(",")]
    if len(values) < 5:
        return None
    try:
        timestamp_value = float(values[0])
        timestamp: int | float = int(timestamp_value) if timestamp_value.is_integer() else timestamp_value
        return {
            "timestamp": timestamp,
            "state": int(float(values[1])),
            "acceleration": float(values[2]),
            "gyroscope": float(values[3]),
            "sound": float(values[4]),
        }
    except (TypeError, ValueError):
        return None


def retrieve_results(n: int = 60) -> list[dict[str, Any]]:
    """Return the last ``n`` valid readings in chronological order."""
    if n <= 0:
        return []
    results = []
    for line in read_from_file().splitlines():
        parsed = parse_sensor_line(line)
        if parsed is not None:
            results.append(parsed)
    return results[-n:]


def _slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "chart"


def generate_graph(x, y, title, title_x, title_y) -> str:
    """Generate a dependency-free SVG chart and return its static-file path."""
    width, height = 860, 300
    left, right, top, bottom = 62, 24, 42, 46
    chart_width = width - left - right
    chart_height = height - top - bottom

    numeric_values: list[float] = []
    for value in y:
        try:
            numeric_values.append(float(value))
        except (TypeError, ValueError):
            numeric_values.append(0.0)

    if numeric_values:
        minimum, maximum = min(numeric_values), max(numeric_values)
        if minimum == maximum:
            padding = max(abs(minimum) * 0.15, 1)
            minimum -= padding
            maximum += padding
    else:
        minimum, maximum = 0.0, 1.0

    def point(index: int, value: float) -> tuple[float, float]:
        x_position = left if len(numeric_values) <= 1 else left + (index / (len(numeric_values) - 1)) * chart_width
        y_position = top + (maximum - value) / (maximum - minimum) * chart_height
        return x_position, y_position

    line_points = " ".join(f"{point(index, value)[0]:.1f},{point(index, value)[1]:.1f}" for index, value in enumerate(numeric_values))
    escaped_title = html.escape(str(title))
    escaped_x = html.escape(str(title_x))
    escaped_y = html.escape(str(title_y))
    grid_lines = []
    for index in range(5):
        y_position = top + (index / 4) * chart_height
        value = maximum - ((maximum - minimum) * index / 4)
        grid_lines.append(
            f'<line x1="{left}" y1="{y_position:.1f}" x2="{width - right}" y2="{y_position:.1f}" class="grid" />'
            f'<text x="{left - 10}" y="{y_position + 4:.1f}" text-anchor="end" class="axis-label">{value:.1f}</text>'
        )
    grid = "".join(grid_lines)
    path = GRAPH_DIR / f"{_slugify(str(title))}.svg"
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" aria-label="{escaped_title}">
  <style>
    .grid {{ stroke: #e9edf4; stroke-width: 1; }}
    .axis-label {{ fill: #8290a6; font: 12px Arial, sans-serif; }}
    .chart-title {{ fill: #25344d; font: 600 15px Arial, sans-serif; }}
    .axis-title {{ fill: #97a2b3; font: 11px Arial, sans-serif; }}
    .trace {{ fill: none; stroke: #ef7465; stroke-width: 3; stroke-linejoin: round; stroke-linecap: round; }}
    .dot {{ fill: #ffffff; stroke: #ef7465; stroke-width: 2; }}
  </style>
  <text x="{left}" y="22" class="chart-title">{escaped_title}</text>
  {grid}
  <line x1="{left}" y1="{top + chart_height}" x2="{width - right}" y2="{top + chart_height}" class="grid" />
  {f'<polyline points="{line_points}" class="trace" />' if line_points else ''}
  {f'<circle cx="{point(len(numeric_values) - 1, numeric_values[-1])[0]:.1f}" cy="{point(len(numeric_values) - 1, numeric_values[-1])[1]:.1f}" r="5" class="dot" />' if numeric_values else ''}
  <text x="{width / 2}" y="{height - 10}" text-anchor="middle" class="axis-title">{escaped_x}</text>
  <text x="15" y="{height / 2}" transform="rotate(-90 15 {height / 2})" text-anchor="middle" class="axis-title">{escaped_y}</text>
</svg>
'''
    path.write_text(svg, encoding="utf-8")
    return path.relative_to(STATIC_DIR).as_posix()


#################################### ALERTS AND SOCKETS ################################


def send_telegram_message(token: str, chat_id: str, message: str) -> bool:
    """Send an optional Telegram notification, returning whether it succeeded."""
    if not token or not chat_id:
        return False
    url = f"https://api.telegram.org/bot{token}/sendMessage"
    try:
        response = requests.post(
            url,
            json={"chat_id": chat_id, "text": message, "parse_mode": "Markdown"},
            timeout=5,
        )
        return response.ok
    except requests.RequestException:
        return False


def generate_message(status_code: int) -> str:
    if status_code not in (3, 4):
        return ""
    resident = personal_info.name or "The resident"
    address = personal_info.address or "the registered address"
    if status_code == 4:
        message = (
            f"URGENT MEDICAL ALERT - LONG LIE ESCALATION\n"
            f"{resident} may have been lying after a fall since {last_fall_time}.\n"
            f"Please call 995 immediately and send an ambulance to {address}."
        )
    else:
        message = (
            f"MEDICAL ALERT\n{resident} has fallen at {last_fall_time}.\n"
            f"Please call 995 to send an ambulance to {address}."
        )
    send_telegram_message(BOT_TOKEN, CHAT_ID, message)
    return message


def _publish_medical_alert(alert_key: str, status_code: int) -> None:
    """Publish state 3 or 4, with an independent five-minute cooldown."""
    global last_fall_time
    normalized_key = alert_key.strip()
    if not normalized_key or status_code not in (3, 4):
        return

    now = time.monotonic()
    if now - alert_last_sent_at[status_code] < ALERT_COOLDOWN_SECONDS:
        return

    alert_last_sent_at[status_code] = now
    last_fall_time = datetime.now().strftime("%d %b %Y, %H:%M")
    message = generate_message(status_code)
    socketio.emit(
        "trigger_reload",
        {
            "reason": "Long lie escalation" if status_code == 4 else "Fall detected",
            "status": status_code,
            "message": message,
            "time": last_fall_time,
        },
    )


def process_sensor_payload(payload: str) -> None:
    """Persist and publish every valid reading in a socket payload."""
    global last_sensor_state

    for raw_line in str(payload).splitlines():
        parsed = parse_sensor_line(raw_line)
        if parsed is None:
            continue
        write_to_file(raw_line)
        last_sensor_state = parsed["state"]
        # Let the per-state cooldown decide whether a repeated alert is due.
        # This allows a continuous stream of state 3/4 values to re-alert after
        # five minutes without requiring an intermediate state change.
        if parsed["state"] in (3, 4):
            _publish_medical_alert(raw_line, parsed["state"])


def run_socket_server() -> None:
    """Listen for the STM32 board without taking down the web app on bind errors."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((SOCKET_IP, SOCKET_PORT))
            server_socket.listen(5)
            print(f"[*] Sensor socket listening on {SOCKET_IP}:{SOCKET_PORT}")
            
            while True:
                client_socket, address = server_socket.accept()
                print(f"[+] Sensor stream connected from {address}")
                buffer = ""
                try:
                    with client_socket:
                        while True:
                            chunk = client_socket.recv(8192)
                            if not chunk:
                                break

                            buffer += chunk.decode("utf-8", errors="replace")
                            complete_lines = buffer.split("\n")
                            buffer = complete_lines.pop()
                            for line in complete_lines:
                                process_sensor_payload(line.rstrip("\r"))

                        # Process a final line if the sender closed without a newline.
                        if buffer.strip():
                            process_sensor_payload(buffer.rstrip("\r"))
                except (ConnectionResetError, BrokenPipeError, OSError) as error:
                    print(f"[-] Sensor stream {address} closed: {error}")
                finally:
                    print(f"[*] Sensor stream disconnected from {address}")
    except OSError as error:
        print(f"[-] Sensor socket unavailable: {error}")


def background_data_listener() -> None:
    """Watch the latest stored value for leading ``3`` or ``4`` markers."""
    while True:
        latest_value = ""
        for line in reversed(read_from_file().splitlines()):
            if line.strip():
                latest_value = line.strip()
                break

        # Some firmware sends a compact status value such as ``3``. The first
        # character is intentionally checked before any CSV parsing so this also
        # works with values like ``3,...``.
        if latest_value and latest_value[0] in ("3", "4"):
            _publish_medical_alert(latest_value, int(latest_value[0]))
        time.sleep(0.25)


#################################### FLASK APP #########################################


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


def _format_sensor_timestamp(value: Any) -> str:
    if value in (None, ""):
        return "Waiting for first reading"
    try:
        number = float(value)
        return f"Sample {int(number)}"
    except (TypeError, ValueError):
        return str(value)


def _status_for_reading(reading: dict[str, Any] | None) -> dict[str, str]:
    if not reading:
        return {"label": "Awaiting sensor data", "detail": "Your monitor is ready to connect.", "tone": "neutral"}
    state = reading.get("state")
    if state == 3:
        return {"label": "Medical alert", "detail": "A possible fall needs attention now.", "tone": "danger"}
    if state in (1, 2):
        return {"label": "Movement detected", "detail": "The monitor is tracking activity.", "tone": "watch"}
    return {"label": "Monitoring normally", "detail": "No unusual movement detected.", "tone": "good"}


def dashboard_context(alert: bool = False) -> dict[str, Any]:
    readings = retrieve_results(1000)
    latest = readings[-1] if readings else None
    acceleration = [reading["acceleration"] for reading in readings]
    graph_path = generate_graph(
        [reading["timestamp"] for reading in readings],
        acceleration,
        "Live movement trace",
        "recent sensor samples",
        "acceleration",
    )
    status = _status_for_reading(latest)
    contacts = [contact for contact in emergency_contacts if any(asdict(contact).values())]
    return {
        "personal": personal_info,
        "contacts": contacts,
        "readings": readings,
        "latest": latest,
        "status": status,
        "graph_path": graph_path,
        "graph_version": int(time.time()),
        "latest_timestamp": _format_sensor_timestamp(latest.get("timestamp") if latest else None),
        "alert": alert or bool(latest and latest.get("state") == 3),
        # Rendering the dashboard must stay side-effect free; Telegram is sent
        # only at the moment process_sensor_payload detects a new fall.
        "alert_message": "",
        "alert_time": last_fall_time,
        "alert_count": sum(1 for reading in readings if reading.get("state") == 3),
    }


@app.route("/")
def index():
    return redirect(url_for("main"))


@app.route("/main", methods=["GET", "POST"])
def main():
    if request.method == "POST":
        return redirect(url_for("main"))
    return render_template("main.html", **dashboard_context(request.args.get("alert") == "1"))


@app.get("/api/status")
def api_status():
    readings = retrieve_results(1)
    latest = readings[-1] if readings else None
    return jsonify(
        {
            "status": _status_for_reading(latest),
            "latest": latest,
            "last_fall_time": last_fall_time,
        }
    )


@app.route("/edit_particulars", methods=["GET", "POST"])
def edit_particulars():
    global personal_info
    saved = False
    if request.method == "POST":
        values = {}
        for field in fields(PersonalInfo):
            values[field.name] = request.form.get(field.name, "").strip()
        with profile_lock:
            personal_info = PersonalInfo(**values)
            _write_json(PARTICULARS_PATH, asdict(personal_info))
        saved = True
    return render_template("edit_particulars.html", personal=personal_info, saved=saved)


@app.route("/edit_emergency_contacts", methods=["GET", "POST"])
def edit_emergency_contacts():
    global emergency_contacts, emergency_contact
    saved = False
    if request.method == "POST":
        try:
            contact_count = max(0, min(int(request.form.get("contact_count", "1")), 20))
        except ValueError:
            contact_count = 1

        updated_contacts: list[EmergencyContact] = []
        for index in range(contact_count):
            contact = EmergencyContact(
                name=request.form.get(f"contact_name_{index}", "").strip(),
                contact=request.form.get(f"contact_phone_{index}", "").strip(),
                relationship=request.form.get(f"contact_relationship_{index}", "").strip(),
            )
            if any(asdict(contact).values()):
                updated_contacts.append(contact)
        if not updated_contacts:
            updated_contacts = [EmergencyContact()]

        with profile_lock:
            emergency_contacts = updated_contacts
            emergency_contact = emergency_contacts[0]
            _write_json(EMERGENCY_CONTACTS_PATH, [asdict(contact) for contact in emergency_contacts])
        saved = True

    return render_template(
        "edit_emergency_contacts.html",
        contacts=emergency_contacts,
        saved=saved,
        personal=personal_info,
    )


#################################### RUN APP ###########################################


if __name__ == "__main__":
    socket_thread = threading.Thread(target=run_socket_server, daemon=True, name="sensor-socket")
    socket_thread.start()
    listener_thread = threading.Thread(target=background_data_listener, daemon=True, name="background-listener")
    listener_thread.start()
    print(f"[*] Starting Fall Detector Base Station on http://{FLASK_IP}:{FLASK_PORT}")
    socketio.run(app, host=FLASK_IP, port=FLASK_PORT, debug=False, allow_unsafe_werkzeug=True)
