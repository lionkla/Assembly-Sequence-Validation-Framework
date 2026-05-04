# Framework zur Validierung von Montagereihenfolgen

Ein physikbasiertes Simulationsframework zur Bewertung und Validierung von Montagereihenfolgen mit PyBullet. Das Framework lädt Komponenten- und Sequenzdefinitionen aus JSON-Dateien, führt eine schrittweise Physiksimulation aus und bewertet jede Sequenz anhand von vier Qualitätskriterien, um ein strukturiertes und reproduzierbares Auswertungsergebnis zu erzeugen.

---

## Inhaltsverzeichnis

1. [Ordnerstruktur](#ordnerstruktur)
2. [Eingabedateien](#eingabedateien)
   - [components.json](#componentsjson)
   - [sequence.json](#sequencejson)
3. [Konfigurierbare Parameter](#konfigurierbare-parameter)
4. [Qualitätskriterien](#qualitätskriterien)
   - [Neue Kriterien hinzufügen](#neue-kriterien-hinzufügen)
   - [Zeiteffizienz](#1-zeiteffizienz)
   - [Stabilität](#2-stabilität)
   - [Zugänglichkeit](#3-zugänglichkeit)
   - [Komplexität](#4-komplexität)
5. [Ausführungsmodi](#ausführungsmodi)
6. [Ausgabedateien](#ausgabedateien)
7. [Kommandozeilenausgabe](#kommandozeilenausgabe)

---

## Ordnerstruktur

Zur Laufzeit wird die folgende Verzeichnisstruktur erwartet. Alle Pfade innerhalb der JSON-Eingabedateien werden **relativ zum Speicherort der Skriptdatei** aufgelöst.

```text
project_root/
│
├── assembly_simulation_framework.py   # Hauptskript
├── components.json                    # Komponentendefinitionen (erforderlich)
├── sequence.json                      # Sequenzdefinition (erforderlich)
│
├── models/                            # 3-D-Geometriedateien (optionaler Unterordner)
│   ├── part_a.urdf
│   ├── part_b.stl
│   └── ...
│
└── assembly_results/                  # Automatisch erzeugtes Ausgabeverzeichnis
    ├── assembly_results_<name>_<timestamp>.json
    ├── assembly_evaluation_results_<timestamp>.csv
    └── assembly_simulation.log
```

Das Verzeichnis `assembly_results/` wird beim ersten Lauf automatisch erstellt. Logdateien werden in das **Arbeitsverzeichnis** geschrieben, aus dem das Skript ausgeführt wird, nicht in `assembly_results/`.

### Unterstützte Geometriedateiformate

Jede Komponente verweist über das Feld `urdf_path` genau auf eine Geometriedatei. Der Lader unterstützt:

| Format | Erweiterung | Hinweise |
|--------|-------------|----------|
| URDF   | `.urdf`     | Vollständige Starrkörperbeschreibung; Masse und Trägheit werden aus der Datei übernommen. Das Feld `mass` in `components.json` überschreibt den URDF-Wert über `changeDynamics`. |
| STL    | `.stl`      | Nur Mesh-Geometrie; das Framework erzeugt Kollisions- und Visualisierungsformen automatisch. Die Skalierung wird über `mesh_scale` gesteuert. |

Andere Formate (OBJ, DAE, SDF) werden vom aktuellen Lader nicht direkt unterstützt. Eingebaute PyBullet-Assets (zum Beispiel `cube.urdf`, `plane.urdf`) können ebenfalls ohne Pfadpräfix aufgelöst werden, weil der PyBullet-Datenpfad automatisch registriert wird.

---

## Eingabedateien

### components.json

Diese Datei definiert die physikalischen Eigenschaften und die Anfangsplatzierung aller Komponenten, die an der Montage teilnehmen. Die Datei muss ein Top-Level-Array `"components"` enthalten.

```json
{
  "components": [
    {
      "id": 1,
      "name": "Base_Plate",
      "urdf_path": "models/base_plate.urdf",
      "mass": 2.0,
      "position": [0.0, 0.0, 0.1],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "dimensions": [0.4, 0.3, 0.05],
      "mesh_scale": [0.001, 0.001, 0.001],
      "connection_points": 3,
      "tools_required": ["screwdriver"]
    }
  ]
}
```

#### Feldreferenz — Komponente

| Feld | Typ | Einheit / Format | Erforderlich | Standardwert | Beschreibung |
|------|-----|------------------|--------------|--------------|--------------|
| `id` | `int` | Positive ganze Zahl, eindeutig pro Montage | Ja | — | Interne Kennung; muss zu den in `sequence.json` verwendeten `component_id`-Werten passen. |
| `name` | `string` | Freitext | Ja | — | Lesbare Bezeichnung; wird in Logmeldungen und Ausgabedateinamen verwendet. |
| `urdf_path` | `string` | Relativer Dateipfad (`.urdf` oder `.stl`) | Ja | — | Pfad zur Geometriedatei, relativ zum Speicherort des Skripts. |
| `mass` | `float` | Kilogramm (kg), > 0 | Ja | — | Masse der Komponente; wird nach dem Laden über `changeDynamics` angewendet. |
| `position` | `[float, float, float]` | Meter (m), `[x, y, z]` im Weltkoordinatensystem | Nein | `[0.0, 0.0, 0.0]` | Anfangsposition in der Simulationswelt. |
| `orientation` | `[float, float, float, float]` | Quaternion `[x, y, z, w]`, Einheitsquaternion | Nein | `[0.0, 0.0, 0.0, 1.0]` | Anfangsorientierung. Ein nicht normierter `w`-Wert > 1.0 wird als Achse-Winkel-Rotation in Grad interpretiert und intern automatisch umgewandelt. |
| `dimensions` | `[float, float, float]` | Meter (m), `[l, w, h]` | Nein | `[1.0, 1.0, 1.0]` | Wird als Ersatzabmessung einer Box verwendet, wenn die Geometriedatei nicht geladen werden kann. |
| `mesh_scale` | `[float, float, float]` | Dimensionslose Skalierungsfaktoren | Nein | `[0.001, 0.001, 0.001]` | Gilt nur für STL-Meshes; wandelt Mesh-Einheiten in Meter um (Standard: mm → m). |
| `connection_points` | `int` | Anzahl (≥ 0) | Nein | `0` | Anzahl physischer Verbindungsschnittstellen; wird von den Kriterien Stabilität und Zugänglichkeit verwendet. |
| `tools_required` | `[string, ...]` | Liste von Werkzeugnamen | Nein | `[]` | Werkzeugkennungen, die für die Montage dieser Komponente benötigt werden; wird vom Kriterium Zeiteffizienz zur Konfliktanalyse verwendet. |

---

### sequence.json

Diese Datei definiert die geordnete Montagereihenfolge als Liste von Schritten. Jeder Schritt verweist über seine `id` auf eine Komponente und legt die Zielpose fest.

```json
{
  "sequence_id": 1,
  "name": "Standard_Assembly_Sequence",
  "description": "Beispielsequenz zur Demonstration",
  "steps": [
    {
      "step_id": 1,
      "component_id": 1,
      "target_position": [0.0, 0.0, 0.1],
      "target_orientation": [0.0, 0.0, 0.0, 1.0],
      "prerequisites": [],
      "tools_used": ["screwdriver"]
    },
    {
      "step_id": 2,
      "component_id": 2,
      "target_position": [-0.15, 0.0, 0.2],
      "target_orientation": [0.0, 0.0, 0.0, 1.0],
      "prerequisites": [1],
      "tools_used": ["screwdriver", "wrench"]
    }
  ]
}
```

#### Feldreferenz — AssemblySequence (oberste Ebene)

| Feld | Typ | Format | Erforderlich | Standardwert | Beschreibung |
|------|-----|--------|--------------|--------------|--------------|
| `sequence_id` | `int` | Positive ganze Zahl | Nein | `1` | Kennung der Sequenz; erscheint in den Ausgabedateien. |
| `name` | `string` | Freitext | Nein | `"Unbenannte Sequenz"` | Wird in Ausgabedateinamen und in der CSV-Zusammenfassung verwendet. |
| `description` | `string` | Freitext | Nein | `""` | Optionale Beschreibung; wird in der Ergebnis-JSON gespeichert. |
| `steps` | `array` | Siehe unten | Ja | — | Geordnete Liste von `AssemblyStep`-Objekten. |

#### Feldreferenz — AssemblyStep

| Feld | Typ | Einheit / Format | Erforderlich | Standardwert | Beschreibung |
|------|-----|------------------|--------------|--------------|--------------|
| `step_id` | `int` | Positive ganze Zahl, eindeutig innerhalb der Sequenz | Ja | — | Kennung des Schritts; wird von `prerequisites` referenziert. |
| `component_id` | `int` | Muss zu einer `id` in `components.json` passen | Ja | — | Wählt aus, welche Komponente in diesem Schritt bewegt wird. |
| `target_position` | `[float, float, float]` | Meter (m), `[x, y, z]` im Weltkoordinatensystem | Ja | — | Endposition der Komponente nach Abschluss dieses Schritts. |
| `target_orientation` | `[float, float, float, float]` | Quaternion `[x, y, z, w]` | Nein | `[0.0, 0.0, 0.0, 1.0]` | Endorientierung; dieselbe Achse-Winkel-Umwandlung wie bei Komponenten wird angewendet. |
| `prerequisites` | `[int, ...]` | Liste von `step_id`-Werten | Nein | `[]` | Schritte, die abgeschlossen sein müssen, bevor dieser Schritt beginnen darf; dient zum Aufbau des Abhängigkeitsgraphen. |
| `tools_used` | `[string, ...]` | Liste von Werkzeugnamen | Nein | `[]` | Werkzeuge, die in diesem Schritt aktiv verwendet werden; wird mit `tools_required` der Komponente für die Konfliktanalyse zusammengeführt. |

---

## Konfigurierbare Parameter

Die folgenden Konstanten und Konstruktorargumente können direkt im Quellcode angepasst werden, um das Simulationsverhalten zu verändern. Eine externe Konfigurationsdatei ist nicht erforderlich.

| Ort im Code | Parameter | Standardwert | Wirkung |
|---|---|---|---|
| `PyBulletSimulator.__init__` | `gui_mode: bool` | `True` | Wechselt zwischen GUI-Modus (interaktives Fenster) und kopflosem DIRECT-Modus. |
| `PyBulletSimulator.__init__` | `gravity: float` | `-9.81` | Gravitationsbeschleunigung in m/s² entlang der Z-Achse. |
| `PyBulletSimulator._initialize_simulation` | `p.setTimeStep(1.0 / 240.0)` | `1/240 s` | Physikalischer Zeitschritt; kleinere Schritte erhöhen die Genauigkeit, verlängern aber die Laufzeit. |
| `PyBulletSimulator.load_component` (`changeDynamics`-Aufruf) | `lateralFriction` | `1.5` | Coulomb-Reibungskoeffizient zwischen der Komponente und anderen Körpern. |
| `PyBulletSimulator.load_component` (`changeDynamics`-Aufruf) | `contactStiffness` / `contactDamping` | `1000` / `100` | Feder-Dämpfer-Parameter für Kontakte, die das Eindringverhalten steuern. |
| `PyBulletSimulator.load_component` (`changeDynamics`-Aufruf) | `linearDamping` / `angularDamping` | `2` / `3` | Geschwindigkeitsdämpfung für frei bewegliche Objekte; reduziert die Einschwingzeit. |
| `PyBulletSimulator.change_dynamics_all_objects` | Alle Dynamikparameter | Höhere Werte nach dem Laden | Wird global angewendet, nachdem alle Komponenten geladen wurden; versteift Kontakte für die eigentliche Bewegungsphase. |
| `PyBulletSimulator.fix_object_after_assembly` | `linearDamping` / `angularDamping` | `8.0` / `8.0` | Hohe Dämpfungswerte, die nach einem Schritt angewendet werden, um platzierte Teile zu fixieren. |
| `PyBulletSimulator.simulate_assembly_step` | `duration: float` | `2.0 s` | Simulationsdauer pro Schritt in Echtzeit; skaliert die Anzahl physikalischer Iterationen. |
| `PyBulletSimulator.simulate_assembly_step` | `liftheight` | `0.05 m` | Vertikaler Sicherheitsabstand, der zu Wegpunkten hinzugefügt wird, um die Komponente vor lateralen Bewegungen über Hindernisse anzuheben. |
| `PyBulletSimulator._compute_min_clearance` | `search_distance: float` | `0.2 m` | Maximale Distanz, bis zu der Nächstpunktabfragen ausgewertet werden; begrenzt den Suchradius für Freiräume. |
| `PyBulletSimulator._check_tool_access_for_pose` | `tool_length: float` | `0.12 m` | Länge des simulierten Werkzeugs, das in den Ray-Cast-Zugänglichkeitsprüfungen verwendet wird. |
| `PyBulletSimulator._check_tool_access_for_pose` | `radial_offset: float` | `0.02 m` | Seitlicher Versatz des Ursprungs des Ray-Casts, um ein Werkzeug mit leicht schräger Annäherung zu simulieren. |
| `TimeEfficiencyCriterion.__init__` | `gamma: float` | `0.5` | Gewichtungsfaktor, der den Parallelisierungsgrad gegen Werkzeugressourcenkonflikte abwägt (Bereich [0, 1]). |
| `AccessibilityCriterion.calculate` | Freiraumschwellen `c_min`, `c1`, `c2`, `c_max` | `0, 10, 40, 80 mm` | Grenzen der trapezförmigen Zugehörigkeitsfunktion für den unscharfen Freiraumindex. |
| `StabilityCriterion.__init__` | `quantitative_weights` / `qualitative_weights` | Siehe Code | TOPSIS-Indikatorgewichte; die Summe über alle fünf Indikatoren muss 1.0 ergeben. |
| `QualityEvaluator._initialize_default_criteria` | `weight` pro Kriterium | `0.25` jeweils | Relatives Gewicht jedes Kriteriums in der finalen Gesamtbewertung. |
| `AssemblyFramework.__init__` | `self.output_dir` | `"assembly_results"` | Verzeichnis, in das alle Ausgabedateien geschrieben werden. |

---

## Qualitätskriterien

Alle Kriterien liefern einen normierten Wert im Intervall **[0.0, 1.0]**, wobei höhere Werte eine bessere Montagequalität anzeigen. Die finale Gesamtbewertung ist das gewichtete arithmetische Mittel aller aktiven Kriterien und wird in `QualityEvaluator.evaluate_sequence` berechnet.

### Neue Kriterien hinzufügen

Ein neues Qualitätskriterium muss den folgenden Vertrag erfüllen:

1. **Von `QualityCriterion` erben** (abstrakte Basisklasse in der Quelldatei).
2. **Drei Methoden implementieren:**
   - `calculate(sequence, components, simulation_data) -> float` — liefert einen Wert in [0.0, 1.0].
   - `get_name() -> str` — liefert eine eindeutige Zeichenkette, die in allen Ausgabedateien als Dictionary-Schlüssel verwendet wird.
   - `get_description() -> str` — liefert eine lesbare Beschreibung, die in der Logausgabe gespeichert wird.
3. **Das Kriterium registrieren** in `QualityEvaluator._initialize_default_criteria` mit `self.add_criterion(YourCriterion(), weight=<float>)`. Das Gewicht ist relativ; der Evaluator normiert über die Summe aller Gewichte. Daher ist nicht die absolute Größe entscheidend, sondern nur das Verhältnis der Gewichte untereinander.

Das Dictionary `simulation_data`, das an `calculate` übergeben wird, enthält alle Ergebnisse auf Schrittniveau, die von `PyBulletSimulator.simulate_assembly_step` erzeugt werden. Dazu gehören Kollisionslisten pro Schritt, minimale Freiraumwerte (`clearance_step_<id>`), Werkzeugzugriffs-Flags (`tool_access_step_<id>`), die globale Kollisionsanzahl sowie der Massenschwerpunkt der Baugruppe.

```python
class MyCriterion(QualityCriterion):
    def calculate(self, sequence, components, simulation_data):
        # eigene Logik hier
        return max(0.0, min(1.0, score))

    def get_name(self):
        return "My_Criterion"

    def get_description(self):
        return "Kurze Erklärung, was dieses Kriterium misst."
```

---

### 1. Zeiteffizienz

**Klasse:** `TimeEfficiencyCriterion` — basiert auf Zhao et al. (2012).

Dieses Kriterium misst, wie effizient die Montage Parallelisierung ausnutzt und Werkzeugressourcenkonflikte vermeidet. Es kombiniert zwei Teilwerte.

Der **Parallelisierungsgrad** wird aus dem Abhängigkeitsgraphen abgeleitet. Die tatsächliche Tiefe des Graphen der Voraussetzungen wird mit theoretischen Grenzen verglichen.

Eine vollständig sequentielle Sequenz erreicht 0; die am stärksten parallelisierbare Struktur erreicht 1.

Die **Werkzeugkonfliktrate** zählt Schrittpaarungen, die mindestens ein Werkzeug gemeinsam nutzen und keine Voraussetzungbeziehung haben, also grundsätzlich gleichzeitig ausgeführt werden könnten. Das Verhältnis konfliktbehafteter Paare zu allen potenziell parallelen Paaren wird von einem Ressourcenfaktor abgezogen.

---

### 2. Stabilität

**Klasse:** `StabilityCriterion` — basiert auf Ma et al. (2015) und verwendet die TOPSIS-Entscheidungsmethode.

Dieses Kriterium erfasst fünf gewichtete Indikatoren in zwei Kategorien.

**Quantitative Indikatoren** (analytisch berechnet):

| Indikator | Gewicht | Berechnung |
|---|---|---|
| `gravity_direction_count` | 0.237 | Anzahl der Schritte, bei denen der Zielpositionsvektor zu mehr als 70 % mit der Z-Achse als Schwerkraftrichtung ausgerichtet ist. |
| `positioning_bases` | 0.186 | Summe der `connection_points` über alle Komponenten. |
| `assembly_relationship` | 0.200 | Anzahl etablierter Kontakte aus der Simulation; alternativ die Schrittzahl, falls keine Kontaktdaten vorliegen. |

**Qualitative Indikatoren** (unscharf bewertet über eine trapezförmige Zugehörigkeitsapproximation):

| Indikator | Gewicht | Berechnung |
|---|---|---|
| `contact_type_quality` | 0.130 | Verbindungsscore pro Schritt, geglättet durch eine symmetrische unscharfe Zugehörigkeitsfunktion. |
| `structural_stability` | 0.248 | Gewichtete Kombination aus drei Teilfaktoren: Höhe des Massenschwerpunkts (40 %), Kollisionsstrafwert (35 %) und geschätzte Gelenkbeanspruchung (25 %), jeweils geglättet durch dieselbe Zugehörigkeitsfunktion. |

Nach der Normalisierung wird jeder Indikator gewichtet und ein TOPSIS-Wert berechnet.

Der Wert beschreibt, wie weit die Sequenz von der schlechtesten möglichen Konfiguration entfernt ist.

---

### 3. Zugänglichkeit

**Klasse:** `AccessibilityCriterion`

Dieses Kriterium kombiniert drei unabhängige Indizes, die jeweils über alle Schritte gemittelt werden.

Der **räumliche Zugänglichkeitsindex** bestraft Kollisionsereignisse, die während jedes Schritts erkannt werden.

Der **Freiraumindex** verwendet eine trapezförmige unscharfe Zugehörigkeitsfunktion auf den minimal gemessenen Freiraum pro Schritt.

Der Freiraum wird von `_compute_min_clearance` über die PyBullet-API `getClosestPoints` in jedem Simulationstakt gemessen.

Der **Werkzeugzugangsindex** wird durch `_check_tool_access_for_pose` bestimmt. Dabei werden Ray-Casts in fünf Anfahrtsrichtungen ausgelöst: +X, −X, +Y, −Y und +Z.

---

### 4. Komplexität

**Klasse:** `ComplexityCriterion` — basiert auf Assembly Sequence Flexibility (ASF).

Dieses Kriterium misst die Dichte struktureller Einschränkungen in der Sequenz. Dazu wird die transitive Hülle des Abhängigkeitsgraphen mit Floyd–Warshall aufgebaut, um alle Paare i,j mit strikter Ordnungsbeziehung zu bestimmen.

Eine vollständig uneingeschränkte Sequenz ohne Voraussetzungen ergibt 1.0. Eine vollständig lineare Kette ergibt einen Wert nahe 0.0.

---

## Ausführungsmodi

Die Simulation kann in zwei Modi ausgeführt werden. Gesteuert wird dies über den Parameter `gui_mode`, der an `AssemblyFramework` übergeben und an `PyBulletSimulator` weitergereicht wird.

### GUI-Modus (`gui_mode=True`, Standard)

PyBullet öffnet ein interaktives 3-D-Ansichtsfenster. Jeder Montageschritt wird in Echtzeit dargestellt, wobei die Bewegung mit einer visuellen Rate von 480 Takten pro Sekunde interpoliert wird (`time.sleep(1/480)` pro Takt). Das Fenster bleibt nach Abschluss der Simulation geöffnet, bis Enter gedrückt wird. Dieser Modus ist für visuelle Inspektion und Debugging gedacht.

### Kopfloser Modus (`gui_mode=False`)

PyBullet verbindet sich im `DIRECT`-Modus ohne grafische Ausgabe. Alle `time.sleep`-Aufrufe werden übersprungen, was die Laufzeit deutlich reduziert. Dieser Modus eignet sich für Batch-Auswertungen, CI-Pipelines oder Serverumgebungen ohne Anzeige.

### Fallback-Modus (Beispieldaten)

Falls `components.json` oder `sequence.json` nicht gefunden wird, fängt die Funktion `main` die `FileNotFoundError` ab und ersetzt die Dateien durch eingebaute Beispieldaten (`create_example_components` und `create_example_sequence`). Die Simulation läuft dann normal im GUI-Modus mit diesen Standarddaten weiter. Das ist nützlich, um die Installation ohne vorbereitete Eingabedateien zu prüfen.

---

## Ausgabedateien

Alle Ausgabedateien werden in das Verzeichnis `assembly_results/` geschrieben. Dieses Verzeichnis ist über `self.output_dir` konfigurierbar.

### 1. JSON-Ergebnisdatei

**Dateinamensmuster:** `assembly_results_<sequence_name>_<YYYYMMDD_HHMMSS>.json`

Für jede bewertete Sequenz wird eine Datei erzeugt. Die Datei enthält den vollständigen Simulationsdatensatz:

```text
{
  "sequence_info": {
    "id": int,
    "name": string,
    "description": string,
    "step_count": int
  },
  "simulation_data": {
    "timestamp": ISO-8601 string,
    "loaded_objects": int,
    "collision_count": int,
    "center_of_mass": [x, y, z],   // Meter
    "total_mass": float,            // kg
    "total_duration": float,        // Sekunden
    "steps": [
      {
        "step_id": int,
        "component_id": int,
        "collisions": [{"time": float, "contactcount": int}, ...],
        "positions": [[x, y, z], ...],
        "orientations": [[x, y, z, w], ...],
        "min_clearance": float,         // Meter
        "tool_access_free": bool,
        "duration": float,              // Sekunden
        "clearance_step_<id>": float,   // Meter
        "tool_access_step_<id>": bool
      }, ...
    ]
  },
  "evaluation_results": {
    "Zeiteffizienz": float,     // [0.0, 1.0]
    "Stabilität": float,        // [0.0, 1.0]
    "Zugänglichkeit": float,    // [0.0, 1.0]
    "Komplexität": float,       // [0.0, 1.0]
    "Gesamtbewertung": float    // gewichtetes Mittel, [0.0, 1.0]
  },
  "timestamp": ISO-8601 string
}
```

### 2. CSV-Zusammenfassungsdatei

**Dateinamensmuster:** `assembly_evaluation_results_<YYYYMMDD_HHMMSS>.csv`

Diese Datei wird von `export_results_to_csv` erzeugt. Sie enthält eine Zeile pro bewerteter Sequenz. Die Spalten bestehen aus festen Metadatenfeldern und zusätzlich aus einer `Score_<name>`-Spalte pro registriertem Kriterium.

| Spalte | Inhalt |
|---|---|
| `Sequence_ID` | Ganzzahlige Sequenzkennung |
| `Sequence_Name` | Name der Sequenz als Zeichenkette |
| `Step_Count` | Anzahl der Montageschritte |
| `Total_Duration` | Gesamte Simulationsdauer in Sekunden |
| `Collision_Count` | Gesamtzahl der Kontaktpunkt-Ereignisse über alle Schritte |
| `Total_Mass` | Summe aller Komponentenmassen in kg |
| `Timestamp` | ISO-8601-Zeitstempel der Auswertung |
| `Score_<criterion>` | Fließkommawert [0.0, 1.0] für jedes registrierte Kriterium |

### 3. Logdatei

**Dateiname:** `assembly_simulation.log` (wird in das Arbeitsverzeichnis geschrieben)

Einfache Textlogdatei im Format `TIMESTAMP - LOGGER - LEVEL - MESSAGE`. Sie protokolliert Ladefehler, die Aktivierung von Ersatzgeometrien und alle Ausnahmen, die während der Schrittsimulation auftreten. Gleichzeitig werden die Einträge auch im Terminal ausgegeben (`StreamHandler`).

---

## Kommandozeilenausgabe

Beim Ausführen von `python assembly_simulation_framework.py` zeigt das Terminal die folgenden Informationen in dieser Reihenfolge an:

```text
Montagereihenfolge Validierungsframework
==================================================
Bewerte Montagereihenfolge: <sequence_name>
{
  "Zeiteffizienz": 0.75,
  "Stabilität": 0.82,
  "Zugänglichkeit": 0.68,
  "Komplexität": 0.60,
  "Gesamtbewertung": 0.71
}

Ergebnisse gespeichert in: assembly_results
Simulation läuft... Drücken Sie Enter zum Beenden.
```

Der JSON-Block ist eine formatiert ausgegebene Darstellung von `evaluation_results`. Alle zwischengespeicherten Details auf Schrittniveau, etwa Positionen, Orientierungen oder Freiraumproben, werden nicht im Terminal ausgegeben, sind aber vollständig in der JSON-Ergebnisdatei enthalten.
