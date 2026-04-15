# SafetyDevice

Wearable safety device using ESP32-C3 that detects gunshots, falls, and 
potential unconsciousness and sends BLE alerts to a Flutter mobile app.

### Setup
## Step 1 — Install VS Code Extensions

1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search **PlatformIO IDE** and install it
4. Restart VS Code when prompted

---

## Step 2 — Install Git

Download and install from https://git-scm.com
Verify it worked by opening a terminal and typing:
git --version

---

## Step 3 — Clone the Project

In the VS Code terminal:
git clone https://github.com/bran-aviles/SafetyDevice.git
cd SafetyDevice

---

## Step 4 — Find Your COM Port
> ⚠️ All `pio` commands must be run in the **PlatformIO Core CLI terminal**,
> not the regular VS Code terminal.
> To open it:
> **VS Code bottom toolbar → PlatformIO alien head icon → Quick Access → 
> Miscellaneous → PlatformIO Core CLI**

In the PlatformIO Core CLI run:
pio device list
Look for **CP210x** or **Silicon Labs** — note the port (e.g. `COM5`).

---

## Step 5 — Set Your COM Port

Open `platformio.ini` and update these two lines with your port:
```ini
monitor_port = COM5
upload_port  = COM5
```

---

## Step 6 — Upload the Code

In the PlatformIO Core CLI run:
pio run -e esp32-c3-devkitm-1 --target upload

---

## Step 7 — Open Serial Monitor

In the PlatformIO Core CLI run:
pio device monitor --port COM5 --baud 115200

You should see:
[BLE] Ready — waiting for connection

---

## Step 8 — Connect via nRF Connect

1. Open nRF Connect on your phone
2. Tap **Scan**
3. Find **SafetyDevice** in the list and tap **Connect**
4. The onboard LED will turn solid blue when connected
5. The serial monitor will print:
```
[BLE] Device connected
```

---

## Step 9 — View Messages from the ESP32

Once connected in nRF Connect:

1. Tap the **Client** tab at the top of the connected device screen
2. Scroll down and tap the service UUID starting with **4fafc201**
3. Find the characteristic starting with **beb5483e**
4. Tap the **down arrow with underline ⬇** next to the characteristic 
   to subscribe to notifications
5. Messages from the ESP32 will now appear directly below the 
   characteristic in real time and look like this:
```json
{"type":"HEART","bpm":72,"baseline_ready":false,"unconscious":false}
```
---

## BLE
- **Device name:** SafetyDevice
- **Service UUID:** 4fafc201-1fb5-459e-8fcc-c5c9c331914b
- **Characteristic UUID:** beb5483e-36e1-4688-b7f5-ea07361b26a8
- **Payload format:** `{"type":"GUNSHOT","confidence":"HIGH","co_occurrence":"NONE"}`

## Project Structure
src/
├── main.cpp
├── audio_detect.h / .cpp      — gunshot detection via microphone saturation
├── heart_monitor.h / .cpp     — MAX30102 BPM, unconsciousness detection
└── alert_manager.h / .cpp     — BLE alerts, cooldown, co-occurrence, LED
include/
└── config.h                   — all shared constants and thresholds
test/
├── test_audio_detect/         — 4 passing tests on ESP32-C3
└── test_heart_monitor/        — 8 passing tests on ESP32-C3
