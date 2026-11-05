# GuardianX Central + Car Hub

TEAM NEXORA, Don Bosco Institute of Technology, Mumbai (Community Engagement Program).

One program for the central ESP32 (DevKit V1). It replaces the older `GuardianX_Central.ino` and `GuardianX_CarHub.ino`.

## What it does

| Part | Job |
|---|---|
| WiFi | Creates **GuardianX-Car** (password `nexora2026`), hub address 192.168.4.1 |
| MPU6050 | Crash when the G-force stays above 5 g for 6 samples within 100 ms (range ±16 g) |
| Button (GPIO32) | During the 10 s countdown: **hold 1 s = cancel**. After an alert: **hold 3 s = "I'm OK"**. Otherwise a short press silences the drowsiness buzzer for 30 s |
| SIM800L | 2 SMS to the emergency contact: location link, then medical info. Sent by a background task, retried up to 3 times |
| GPS | Location for the SMS and the dashboard |
| Camera 1 | Read every second: who is driving, drowsiness level |
| Dashcam | Told to save its clip the moment a crash is detected |
| Buzzer | Crash countdown: fast beeping. Drowsiness: level 2 and 3 |
| LCD | Driver name, eye status, GPS fix; countdown and alert screens |
| Dashboard | `GET /status`, `POST /driver`, `POST /test` |

## Who gets the crash SMS

1. The relative's number of the **recognised driver** (from the dashboard's driver registration).
2. If no driver is recognised: the **Backup emergency number** from the dashboard (Settings > Car network).
3. If neither: `DEFAULT_EMERGENCY_NUMBER` at the top of the sketch.

Numbers can be typed any way (`98765 43210`, `09876543210`, `+91 98765-43210`); the hub turns them into `+919876543210`.

## SMS fix (October 2026)

**What went wrong.** When the dashboard asked for a test SMS, the hub sent it *inside* the web request and, while waiting for the SIM800L (up to 30 s), it also answered the dashboard's status requests from inside that wait. That broke the reply to the original request, so the dashboard said *"SIM800L didn't answer / hub offline"* even though the hub was fine.

**What changed.**
- All SIM800L work runs in its own task. Web requests return at once, and the dashboard follows the result.
- Before every SMS the hub sets text mode again. A SIM800L that restarted after a power dip forgets text mode, and then every SMS fails.
- It waits up to 30 s for network registration, retries crash alerts 3 times, and explains failures in plain English. For example: no balance or SMS pack, SMS centre missing, Jio SIM (no 2G), or *"SIM800L restarted while transmitting = power supply too weak"*.
- The SIM800L is checked every 30 s: signal, network, operator, supply voltage, SMS centre. The circuit page shows any problem.

## Libraries

Arduino IDE → **Tools → Manage Libraries**, install:
- **hd44780** by Bill Perry
- **TinyGPSPlus** by Mikal Hart

Board: **ESP32 Dev Module** (esp32 by Espressif, version 2.x or 3.x).

## Settings at the top of the sketch

| Setting | Default | Change if |
|---|---|---|
| `DEFAULT_EMERGENCY_NUMBER` | `""` | Last-resort number, e.g. `"+919876543210"` (easier: the dashboard's Backup number) |
| `CRASH_G` | `5.0` | Crash threshold |
| `BUZZER_IS_ACTIVE` | `true` | Set `false` for a passive piezo that only clicks |

## Serial Monitor commands (115200, line ending "Newline")

| Type | Does |
|---|---|
| `c` | Fake crash: starts the 10 s countdown (uses a test location if GPS has no fix) |
| `x` | Cancel the countdown |
| `r` | Clear an alert ("I'm OK") |
| `0` `1` `2` `3` | Test the drowsiness buzzer at that level for 15 s |
| `g` | Live G-force printout on/off |
| `m` | Check the SIM800L (signal, network, voltage, SMS centre) |
| `t` | Send a test SMS to the backup number |
| `s` | Status of everything: WiFi, MPU, GPS, SIM, LCD, cameras, driver, emergency numbers |

## SIM800L checklist (most common reasons an SMS doesn't arrive)

| Check | Good value |
|---|---|
| Supply | 3.7–4.2 V **that can give 2 A** (Li-ion cell or buck converter at 4.0 V), 1000 µF capacitor at VCC/GND, short thick wires. Never the ESP32's 3.3 V / 5 V pin |
| Ground | SIM800L GND joined to ESP32 GND |
| SIM | Airtel, Vi or BSNL (2G). **Jio does not work** (no 2G). Balance or an SMS pack, no PIN lock |
| Signal | CSQ 10–31 with the antenna fitted |
| Wiring | SIM TX → GPIO26, SIM RX ← GPIO27 via 1 k / 2.2 k divider |

If something is unclear, upload `GuardianX_SIM800_Test.ino` for a full step-by-step report.
