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