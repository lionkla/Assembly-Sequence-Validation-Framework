# Konfigurationsdateien für das Montagereihenfolge-Framework

## 1. Beispiel-Bauteildefinition (components_example.json)
<!--
  Datenklasse für Bauteile in der Montagesimulation.

  Attributes:
      id: Eindeutige Identifikation des Bauteils
      name: Bezeichnung des Bauteils
      urdf_path: Pfad zur URDF oder STL-Datei
      mass: Masse des Bauteils in kg
      position: Startposition [x, y, z]
      orientation: Startorientierung [x, y, z, w] (Quaternion)
      dimensions: Abmessungen [length, width, height]
      connection_points: Anzahl Verbindungspunkte zu anderen Bauteilen
      tools_required: Benötigte Werkzeuge für die Montage

  Datentypen:
    id: int
    name: str
    urdf_path: str
    mass: float
    position: List[float] = field(default_factory=lambda: [0, 0, 0])
    orientation: List[float] = field(default_factory=lambda: [0, 0, 0, 1])
    dimensions: List[float] = field(default_factory=lambda: [1, 1, 1])
    connection_points: int = 0
    tools_required: List[str] = field(default_factory=list)

-->
```json
{
  "components": [
    {
      "id": 1,
      "name": "Grundplatte",
      "urdf_path": "cube.urdf",
      "mass": 2.0,
      "position": [0, 0, 0.1],
      "orientation": [0, 0, 0, 1],
      "dimensions": [0.4, 0.3, 0.05],
      "connection_points": 3,
      "tools_required": ["screwdriver"]
    },
    {
      "id": 2,
      "name": "Träger_Links",
      "urdf_path": "cube.urdf",
      "mass": 0.5,
      "position": [-0.15, 0, 0.2],
      "orientation": [0, 0, 0, 1],
      "dimensions": [0.05, 0.25, 0.15],
      "connection_points": 2,
      "tools_required": ["screwdriver", "wrench"]
    },
    {
      "id": 3,
      "name": "Träger_Rechts",
      "urdf_path": "cube.urdf",
      "mass": 0.5,
      "position": [0.15, 0, 0.2],
      "orientation": [0, 0, 0, 1],
      "dimensions": [0.05, 0.25, 0.15],
      "connection_points": 2,
      "tools_required": ["screwdriver", "wrench"]
    }
  ]
}
```

## 2. Beispiel-Montagesequenz (sequence_example.json)
<!--
  Datenklasse für einzelne Montageschritte.

  Attributes:
      step_id: Eindeutige Schritt-ID
      component_id: ID des zu montierenden Bauteils
      target_position: Zielposition nach Montage
      target_orientation: Zielorientierung nach Montage
      prerequisites: Voraussetzungen (andere Schritte)
      tools_used: Verwendete Werkzeuge

  Datentypen:
    step_id: int
    component_id: int
    target_position: List[float]
    target_orientation: List[float] = field(default_factory=lambda: [0, 0, 0, 1])
    prerequisites: List[int] = field(default_factory=list)
    tools_used: List[str] = field(default_factory=list)
-->

```json
{
  "sequence_id": 1,
  "name": "Optimierte_Montagereihenfolge",
  "description": "Zeitoptimierte Sequenz mit minimalen Werkzeugwechseln",
  "steps": [
    {
      "step_id": 1,
      "component_id": 1,
      "target_position": [0, 0, 0.1],
      "target_orientation": [0, 0, 0, 1],
      "prerequisites": [],
      "tools_used": ["screwdriver"],
    },
    {
      "step_id": 2,
      "component_id": 2,
      "target_position": [-0.15, 0, 0.2],
      "target_orientation": [0, 0, 0, 1],
      "prerequisites": [1],
      "tools_used": ["screwdriver", "wrench"],
    }
  ]
}
```

## 3. Verwendungsanleitung

### Installation der Abhängigkeiten:
```bash
pip install pybullet numpy pandas
```

### Grundlegende Nutzung:
```python
# Framework initialisieren
framework = AssemblyFramework(gui_mode=True)

# Bauteile laden
framework.load_components_from_file("components_example.json")

# Montagereihenfolge laden  
framework.load_sequence_from_file("sequence_example.json")

# Bewertung durchführen
sequence = framework.sequences[0]
results = framework.run_full_evaluation(sequence)

# Bericht generieren
report = framework.generate_report(sequence, results)
print(report)
```

### Erweiterte Nutzung - Mehrere Sequenzen vergleichen:
```python
# Mehrere Sequenzen laden
sequences = []
for i in range(1, 4):
    framework.load_sequence_from_file(f"sequence_{i}.json")
    sequences.append(framework.sequences[-1])

# Vergleich durchführen
comparison = framework.compare_sequences(sequences)
print(f"Beste Sequenz: {comparison['best_sequence']}")
```

## 4. Eigene Gütekriterien hinzufügen

```python
class CustomCriterion(QualityCriterion):
    def calculate(self, sequence, components, simulation_data):
        # Ihre Bewertungslogik hier
        return score  # 0.0 - 1.0

    def get_name(self):
        return "Mein_Kriterium"

    def get_description(self):
        return "Beschreibung des Kriteriums"

# Kriterium hinzufügen
framework.evaluator.add_criterion(CustomCriterion(), weight=0.3)
```

## 5. Ausgabeformate

Das Framework generiert automatisch:
- JSON-Dateien mit detaillierten Ergebnissen
- CSV-Dateien für statistische Analyse
- Textbasierte Bewertungsberichte
- Vergleichsanalysen zwischen Sequenzen

## 6. Wissenschaftliche Auswertung

Für die Masterarbeit werden folgende Daten erfasst:
- Simulationszeiten und Kollisionsdaten
- Bewertungen nach allen Gütekriterien
- Vergleichsmetriken zwischen Sequenzen
- Statistische Auswertungsdaten im CSV-Format

Diese können direkt in Statistiksoftware wie R, SPSS oder Python/Pandas importiert werden.

## Implementierung der PyBullet Simulation

Die Implementierung der Simulationsumgebung erfolgt in der Klasse PyBulletSimulator. Dadurch bleibt der Simulator austauschbar. Sie verwaltet die Verbindung zum Physik-Server, die grundlegenden Simulationsparameter sowie das Laden und Bewegen der Bauteile während der Montagereihenfolge.
Die Initialisierung schafft eine standardisierte Umgebung. Die Klasse wählt zwischen grafischer GUI-Verbindung und „headless“-Modus. Der GUI-Modus aktiviert die „Debug“-Visualisierung. „Headless“-Modus spart Rechenressourcen für Batch-Simulationen. Der Suchpfad für URDF-Dateien registriert PyBulletData. Gravitation setzt sich auf -9.81 m/s² in Z-Richtung. Dieser Wert entspricht der Erdbeschleunigung. Er simuliert reale Montagebedingungen genau. Der Zeitschritt beträgt 1/240 Sekunden. Diese Frequenz gewährleistet stabile Physikberechnungen bei hoher Genauigkeit. Eine Bodenebene aus plane.urdf dient als stabiler Boden. Diese Werte sorgen für reproduzierbare Starts. Sie minimieren Variabilität zwischen Simulationen.
Das Laden von Bauteilen priorisiert URDF-Modelle. Die Klasse platziert Bauteile an Startposition und -orientierung aus Component. Sie setzt Dämpfungen: Lineare auf 0.1 und rotatorische auf 0.1. Diese Werte dämpfen Schwingungen realistisch. Sie verhindern unphysikalische Oszillationen ohne die Dynamik zu verzerren. Die PyBullet-ID speichert sich zentral. Bei STL-Dateien skaliert sie „Meshes“ via dimensions. Visuelle Farbe wählt sich als Blau (0.5, 0.5, 0.8, 1.0). Diese Wahl verbessert die Sichtbarkeit. Sie differenziert Bauteile optisch ohne Realismus zu fordern. Bei fehlgeschlagenem Laden aktiviert sich ein „Fallback“. Die Klasse erzeugt mehrere Boxen. Masse und Position übernimmt sie aus Component. Kollisions- und visuelle Formen entstehen getrennt. Farbe bleibt Blau. Dieser Mechanismus hält die Simulation lauffähig und wird vom Autor zum Testen des Programmcodes genutzt. Dies vermeidet Abstürze durch defekte Modelle. Vereinfachte Geometrie erlaubt schnelle Tests. Genauigkeit leidet minimal, da Abmessungen erhalten bleiben.
Die Simulation von Montageschritten verwendet lineare Positionsinterpolation und Quaternion-Vektoren für Orientierung. Standarddauer beträgt 5 Sekunden. Anzahl Schritte ergibt sich aus Dauer mal 240 Hz. Jeder Mikroschritt setzt Position/Orientierung und führt Physik aus. Kollisionen erfasst getContactPoints pro Bauteil. GUI-Modus pausiert mit 1/240 Sekunden. Diese Werte balancieren Geschwindigkeit und Präzision. Lineare Pfade approximieren reale Roboterbahnen. Hohe Frequenz fängt Kollisionen fein auf. Daten wie Trajektorien und Dauer fließen in Bewertungen ein. Aggregierte Daten liefert get_simulation_data. Sie zählt Objekte und Kollisionen. Schwerpunkt berechnet sich massengewichtet aus Positionen und Massen. Diese Metriken unterstützen Stabilitäts- und Zugänglichkeitskriterien. Reset_simulation leert Objekte und lädt Boden neu. Disconnect schließt den Server. Diese Funktionen gewährleisten saubere Neustarts. Ressourcen bleiben frei. Der gewählte Aufbau maximiert Robustheit und Flexibilität. Hartcodierte Werte wie Gravitation, Zeitschritt und Dämpfungen orientieren sich an physikalischen Standards.
