## Dieses Repository begleitet das Buch MeshRadio – Embedded LoRa Mesh Engineering.
## https://nerdverlag.com  Buch MeshRadio – Embedded LoRa Mesh Engineering
## fritz@nerdverlag.com

<p align="center">
<img src="MeshRadioBonus/docs/coverLoRa.png" width="400">
</p>

Der Fokus liegt auf Engineering-Verständnis, Architekturentscheidungen und reproduzierbarer Embedded-Entwicklung — nicht auf schnellen Copy-Paste-Lösungen.

# 🚀 MeshRadio – Embedded LoRa Mesh Engineering

Vom experimentellen LoRa-Chat zum professionellen Mesh-System

LoRa ermöglicht enorme Reichweiten bei minimalem Energieverbrauch — doch stabile Mesh-Netzwerke entstehen nicht durch Zufall. Sie entstehen durch Engineering.

MeshRadio – Embedded LoRa Mesh Engineering führt Schritt für Schritt durch die Entwicklung eines eigenen Embedded-LoRa-Mesh-Systems: von den ersten Funkpaketen über Routing-Algorithmen, Control-Plane-Design und sichere Datenübertragung bis hin zu einem robusten, profi-level stabilen Gesamtsystem.

Dieses Projekt ist kein Installationshandbuch und kein Produktmanual.
Es ist ein Engineering-Guide.

## 🎯 Fokus des Projekts

Systematischer Aufbau eines LoRa-Mesh-Stacks
Stabile Routing-Logik und Node-Rollen
Embedded-Architektur für reale Funkbedingungen
Sicherheit mit AES-CCM und Replay-Schutz
Praktische Entwicklung direkt auf ESP32-basierter Hardware
Alle Konzepte werden anhand realer, getesteter und lauffähiger Software erklärt.

## 📂 Repository-Struktur

Das begleitende GitHub-Repository enthält mehr als 40 sauber strukturierte Source-Pakete, die einzelne Entwicklungsstufen des Systems repräsentieren.

Ziel ist es, jede Phase nachvollziehbar zu machen:

verstehen
testen
erweitern
weiterentwickeln

Jedes Modul baut logisch auf dem vorherigen auf.

# ⚠️ Wichtiger Hinweis zur Projektstruktur (VS Code)

Dieses Repository enthält mehr als 40 eigenständige Source-Module.

Jedes Verzeichnis ist als separates Projekt aufgebaut.

➡️ Daher muss jedes Modul einzeln in Visual Studio Code geöffnet werden.

✔️ Richtig

VS Code → File → Open Folder → MESH-RADIO-KAP7/moduleXX

❌ Nicht empfohlen

Gesamtes Repository als ein einziges VS Code Projekt öffnen

Dies führt typischerweise zu:

fehlerhaften Build-Konfigurationen

falschen Include-Pfaden

Konflikten zwischen Build-Systemen

nicht reproduzierbaren Ergebnissen

Hintergrund

Die Module repräsentieren einzelne Entwicklungsstufen des MeshRadio-Engineering-Prozesses und besitzen jeweils eigene Build-Umgebungen.

Die isolierte Nutzung stellt sicher, dass:

Builds reproduzierbar bleiben

Abhängigkeiten korrekt aufgelöst werden

jede Entwicklungsstufe unabhängig getestet werden kann

## 🧩 Zielsetzung

Das Ergebnis ist kein theoretisches Beispiel, sondern ein praxisnahes Framework — geeignet für:

Entwickler
Maker
Forschung
industrielle Prototypen
Funkexperimente

## ⚠️ Haftungsausschluss

Dieses Projekt wird ohne jegliche Garantie bereitgestellt.

Die Nutzung erfolgt auf eigene Verantwortung.
Der Autor übernimmt keine Haftung für Schäden, Datenverlust oder Fehlfunktionen, die aus der Verwendung der Software entstehen.

## 🧾 Garantie / Gewährleistung

Es besteht keine ausdrückliche oder stillschweigende Garantie, einschließlich:
Fehlerfreiheit
Eignung für einen bestimmten Zweck
dauerhafte Stabilität unter allen Bedingungen

## © Copyright

Copyright (c) 2026
Friedrich Riedhammer DJ2RF
https://nerdverlag.com

Alle Rechte vorbehalten, sofern nicht durch die Lizenz anders geregelt.

## 📜 Lizenz

Siehe Datei:

