# English at Bottom 

# MeshRadio Console GUI

Live Monitoring- und Steuerkonsole für **MeshRadio Nodes** über die serielle Schnittstelle.

Das Tool visualisiert Sensordaten (Temperatur, Druck, Luftfeuchte, Batterie) und erlaubt das
Senden von Befehlen an MeshRadio Nodes innerhalb des **AWAKE TX-Fensters**.

Die GUI ist speziell für **MeshRadio Firmware mit SENSOR:AWAKE WX Nachrichten** ausgelegt.

---

# Features

### Live Visualisierung
- Temperatur Plot
- Luftdruck Plot
- Luftfeuchte Plot
- Batterie Spannung
- Batterie Prozent

### Node Steuerung
- Direkter **SEND Command**
- **ACK Unterstützung**
- Zielnode frei wählbar

### Quick Commands
Buttons für typische Aktionen:


RELAY ON
RELAY OFF
RELAY TOGGLE
STATUS?


### TX Window Protection

Nach jeder


SENSOR:AWAKE


Nachricht öffnet sich ein **TX Fenster** (standard: 30 Sekunden).

Nur in diesem Zeitfenster werden Commands akzeptiert.

Visualisierung:


🟢 grün = TX erlaubt
⚪ grau = TX gesperrt


---

### One-Shot Auto Command

Ein Befehl kann **automatisch beim nächsten AWAKE** gesendet werden.

Workflow:

1. Command eingeben
2. `ARM (next AWAKE)` drücken
3. beim nächsten SENSOR:AWAKE wird der Befehl automatisch gesendet

Der Command wird **nur einmal** ausgeführt.

Statusanzeige:


DISARMED
ARMED (waiting AWAKE)
SENT ✓


---

# Unterstützte Log Nachrichten

Beispiel MeshRadio Ausgabe:


DATA delivered ✅ from=DL1ABCF rssi=-38
"SENSOR:AWAKE WX t=23.02C p=978hPa rh=42.47% bat=4098mV bat=87%"


Die GUI extrahiert daraus:

| Feld        | Beispiel|
|-------------|---------|
| Temperatur  | 23.02°C |
| Druck       | 978 hPa |
| Luftfeuchte | 42.47 % |
| Batterie    | 4098 mV |
| Batterie    | 87 %    |

---

# Installation

## Python Version

Empfohlen:


Python 3.10+


---

## Benötigte Pakete


pyserial
matplotlib


Installation:


pip install pyserial matplotlib


---

# MSYS2 / UCRT64 Hinweis

Wenn Python aus **MSYS2** verwendet wird:


pacman -S mingw-w64-ucrt-x86_64-python-pyserial
pacman -S mingw-w64-ucrt-x86_64-python-matplotlib


---

# Programm starten

Standard:


python meshradio_console_gui.py


oder explizit:


python meshradio_console_gui.py --port COM16


---

# CLI Parameter

| Parameter      | Beschreibung           |
|----------------|------------------------|
| `--port`       | Serielle Schnittstelle |
| `--baud`       | Baudrate               |
| `--timeout`    | Serial timeout         |
| `--window`     | TX Zeitfenster         |
| `--dst`        | Default Zielnode       |
| `--max-points` | Anzahl Plotpunkte      |

---

### Beispiel


python meshradio_console_gui.py
--port COM16
--baud 115200
--window 30
--dst DL1ABCF


---

# GUI Bereiche

## 1. Plot Fenster

Zeigt:


Temperatur
Druck
Luftfeuchte
Batterie


Alle Werte werden live aktualisiert.

---

## 2. Control Panel

### TX Window Anzeige

Zeigt verbleibende Zeit nach `SENSOR:AWAKE`.

Beispiel:


23s (open)


---

### SEND Command

Manueller Befehl:


send <DST> <ACK> <COMMAND>


Beispiel:


send DL1ABCF 1 CMD:STATUS?


---

### Quick Buttons

Schnelle Relaissteuerung:


RELAY ON
RELAY OFF
RELAY TOGGLE
STATUS?


---

### Auto Command

Ein Command kann für den nächsten AWAKE vorbereitet werden.

Beispiel:


CMD:RELAY ON


Dann:


ARM (next AWAKE)


Beim nächsten Wakeup wird automatisch gesendet.

---

# Troubleshooting

## Kein Serial Port


[ERR] Cannot open COM16


Lösung:

Port prüfen:


Device Manager → COM Ports


---

## Keine Daten im Plot

Mögliche Ursachen:

- falscher COM Port
- falsche Baudrate
- Firmware sendet keine `SENSOR:AWAKE`

---

## Python Modul fehlt


ModuleNotFoundError: serial


Installieren:


pip install pyserial


---

# Projektstruktur


MeshRadioBonus
├─ visualisierung
│ ├─ meshradio_console_gui.py
│ └─ README.md


---

# Lizenz

Open Source für Amateurfunk- und Lernzwecke.


(c) 2026 Friedrich Riedhammer
fritz@nerdverlag.com
https://nerdverlag.com


---

# Zukunftsideen

Mögliche Erweiterungen:

- RSSI Anzeige
- SNR Anzeige
- Node Liste
- Mesh Topology Map
- CSV Logging
- MQTT Export
- Web Dashboard


DJ2RF Fritz


# English:

# MeshRadio Console GUI

Live monitoring and control console for **MeshRadio nodes** via serial interface.

The tool visualizes environmental sensor data (temperature, pressure, humidity, battery)
and allows sending commands to MeshRadio nodes within the **AWAKE TX window**.

The GUI is designed for **MeshRadio firmware that outputs `SENSOR:AWAKE WX` messages**.

---

# Features

## Live Data Visualization

The program plots sensor data in real time:

- Temperature
- Air pressure
- Relative humidity
- Battery voltage
- Battery percentage

Two live graphs show the current node data.

---

## Node Control

Commands can be sent directly to nodes.

Supported features:

- manual command entry
- ACK support
- configurable destination node

Example command:


send DL1ABCF 1 CMD:STATUS?


---

## Quick Command Buttons

Frequently used commands are available as buttons:


RELAY ON
RELAY OFF
RELAY TOGGLE
STATUS?


These commands are sent with **ACK enabled**.

---

# TX Window Protection

After each


SENSOR:AWAKE


message the node opens a **TX window** (default: 30 seconds).

Commands are only allowed during this window.

Visual indicator:


🟢 green = TX allowed
⚪ grey = TX closed


The remaining time is shown in seconds.

---

# One-Shot Auto Command

A command can be prepared and automatically transmitted on the **next AWAKE event**.

Workflow:

1. Enter command
2. Click **ARM (next AWAKE)**
3. Wait for next `SENSOR:AWAKE`
4. Command is transmitted automatically

The command is executed **only once**.

Status indicators:


DISARMED
ARMED (waiting AWAKE)
SENT ✓


---

# Supported Log Format

Example MeshRadio output:


DATA delivered ✅ from=DL1ABCF rssi=-38
"SENSOR:AWAKE WX t=23.02C p=978hPa rh=42.47% bat=4098mV bat=87%"


The GUI extracts:

| Field              | Example |
|--------------------|---------|
| Temperature        | 23.02°C |
| Pressure           | 978 hPa |
| Humidity           | 42.47 % |
| Battery voltage    | 4098 mV |
| Battery percentage | 87 %    |

---

# Installation

## Python Version

Recommended:


Python 3.10 or newer


---

## Required Python packages


pyserial
matplotlib


Install via pip:


pip install pyserial matplotlib


---

# MSYS2 / UCRT64 Note

If Python is installed via **MSYS2**, install the packages using:


pacman -S mingw-w64-ucrt-x86_64-python-pyserial
pacman -S mingw-w64-ucrt-x86_64-python-matplotlib


---

# Running the Program

Default start:


python meshradio_console_gui.py


Example specifying the serial port:


python meshradio_console_gui.py --port COM16


---

# Command Line Parameters

| Parameter      | Description                          |
|----------------|--------------------------------------|
| `--port`       | Serial port                          |
| `--baud`       | Baud rate                            |
| `--timeout`    | Serial timeout                       |
| `--window`     | TX window duration                   |
| `--dst`        | Default destination node             |
| `--max-points` | Number of points stored for plotting |

---

## Example


python meshradio_console_gui.py
--port COM16
--baud 115200
--window 30
--dst DL1ABCF


---

# GUI Overview

## 1. Plot Window

Displays live sensor data:

- Temperature
- Pressure
- Humidity
- Battery voltage
- Battery percentage

The plots update continuously as new data arrives.

---

## 2. Control Panel

### TX Window Indicator

Shows how long the node will accept commands after a wakeup.

Example:


23s (open)


---

### Manual SEND Command

Commands follow the MeshRadio CLI format:


send <DST> <ACK> <COMMAND>


Example:


send DL1ABCF 1 CMD:STATUS?


---

### Quick Control Buttons

Provides instant relay control:


RELAY ON
RELAY OFF
RELAY TOGGLE
STATUS?


---

### Auto Command

Prepare a command that will be transmitted automatically at the next wake event.

Example:


CMD:RELAY ON


Then click:


ARM (next AWAKE)


---

# Troubleshooting

## Serial Port cannot be opened

Error:


[ERR] Cannot open COM16


Check the correct port using:


Device Manager → Ports (COM & LPT)


---

## No data in the plot

Possible causes:

- wrong serial port
- wrong baud rate
- firmware does not output `SENSOR:AWAKE`

---

## Missing Python module

Example error:


ModuleNotFoundError: serial


Install:


pip install pyserial


---

# Project Structure


MeshRadioBonus
├─ visualisierung
│ ├─ meshradio_console_gui.py
│ └─ README.md


---

# License

Open source.


(c) 2026 Friedrich Riedhammer

fritz@nerdverlag.com

https://nerdverlag.com


---

# Future Ideas

Possible future features:

- RSSI display
- SNR display
- Node list
- Mesh topology visualization
- CSV data logging
- MQTT export
- Web dashboard


Fritz DJ2RF