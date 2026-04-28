# Assembly Sequence Validation Framework

A physics-based simulation framework for evaluating and validating assembly sequences using PyBullet. The framework loads component and sequence definitions from JSON files, runs a step-by-step physics simulation, and scores each sequence across four quality criteria to produce a structured, reproducible evaluation result.

---

## Table of Contents

1. [Folder Structure](#folder-structure)
2. [Input Files](#input-files)
   - [components.json](#componentsjson)
   - [sequence.json](#sequencejson)
3. [Configurable Parameters](#configurable-parameters)
4. [Quality Criteria](#quality-criteria)
   - [Adding New Criteria](#adding-new-criteria)
   - [Time Efficiency](#1-time-efficiency-zeiteffizienz)
   - [Stability](#2-stability-stabilität)
   - [Accessibility](#3-accessibility-zugänglichkeit)
   - [Complexity](#4-complexity-komplexität)
5. [Execution Modes](#execution-modes)
6. [Output Files](#output-files)
7. [Command-Line Output](#command-line-output)

---

## Folder Structure

The following directory layout is expected at runtime. All paths within the input JSON files are resolved **relative to the script file location**.

```
project_root/
│
├── assembly_simulation_framework.py   # Main script
├── components.json                    # Component definitions (required)
├── sequence.json                      # Sequence definition (required)
│
├── models/                            # 3-D geometry files (optional subfolder)
│   ├── part_a.urdf
│   ├── part_b.stl
│   └── ...
│
└── assembly_results/                  # Auto-generated output directory
    ├── assembly_results_<name>_<timestamp>.json
    ├── assembly_evaluation_results_<timestamp>.csv
    └── assembly_simulation.log
```

The `assembly_results/` directory is created automatically on the first run. Log files are written to the **working directory** from which the script is executed, not into `assembly_results/`.

### Accepted Geometry File Formats

Each component references exactly one geometry file via the `urdf_path` field. The loader supports:

| Format | Extension | Notes |
|--------|-----------|-------|
| URDF   | `.urdf`   | Full rigid-body description; mass and inertia taken from the file. The `mass` field in `components.json` overrides the URDF value via `changeDynamics`. |
| STL    | `.stl`    | Mesh-only; the framework creates collision and visual shapes automatically. Scale is controlled via `mesh_scale`. |

No other formats (OBJ, DAE, SDF) are directly supported by the current loader. PyBullet's built-in assets (e.g., `cube.urdf`, `plane.urdf`) are also resolvable without a path prefix because the PyBullet data path is registered automatically.

---

## Input Files

### components.json

Defines the physical properties and initial placement of every component that participates in the assembly. The file must contain a top-level `"components"` array.

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

#### Field Reference — Component

| Field              | Type              | Unit / Format                              | Required | Default           | Description |
|--------------------|-------------------|--------------------------------------------|----------|-------------------|-------------|
| `id`               | `int`             | Positive integer, unique per assembly      | Yes      | —                 | Internal identifier; must match the `component_id` values used in `sequence.json`. |
| `name`             | `string`          | Free text                                  | Yes      | —                 | Human-readable label; used in log messages and output filenames. |
| `urdf_path`        | `string`          | Relative file path (`.urdf` or `.stl`)     | Yes      | —                 | Path to the geometry file, resolved relative to the script location. |
| `mass`             | `float`           | Kilograms (kg), > 0                        | Yes      | —                 | Component mass; applied via `changeDynamics` after loading. |
| `position`         | `[float, float, float]` | Meters (m), `[x, y, z]` in world frame | No       | `[0.0, 0.0, 0.0]` | Initial spawn position in the simulation world. |
| `orientation`      | `[float, float, float, float]` | Quaternion `[x, y, z, w]`, unit quaternion | No | `[0.0, 0.0, 0.0, 1.0]` | Initial orientation. A non-unit `w` component > 1.0 is interpreted as an axis-angle rotation in degrees (auto-converted internally). |
| `dimensions`       | `[float, float, float]` | Meters (m), `[l, w, h]`              | No       | `[1.0, 1.0, 1.0]` | Used as fallback box extents when the geometry file cannot be loaded. |
| `mesh_scale`       | `[float, float, float]` | Dimensionless scale factors            | No       | `[0.001, 0.001, 0.001]` | Applied to STL meshes only; converts mesh units to meters (default: mm → m). |
| `connection_points`| `int`             | Count (≥ 0)                                | No       | `0`               | Number of physical connection interfaces; used by the Stability and Accessibility criteria. |
| `tools_required`   | `[string, ...]`   | List of tool-name strings                  | No       | `[]`              | Tool identifiers needed to install this component; used by the Time Efficiency criterion for conflict detection. |

---

### sequence.json

Defines the ordered assembly sequence as a list of steps. Each step refers to a component by its `id` and specifies the target pose.

```json
{
  "sequence_id": 1,
  "name": "Standard_Assembly_Sequence",
  "description": "Example sequence for demonstration",
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

#### Field Reference — AssemblySequence (top level)

| Field         | Type     | Format         | Required | Default                | Description |
|---------------|----------|----------------|----------|------------------------|-------------|
| `sequence_id` | `int`    | Positive integer | No     | `1`                    | Identifier for the sequence; appears in output files. |
| `name`        | `string` | Free text       | No       | `"Unbenannte Sequenz"` | Used in output filenames and the CSV summary. |
| `description` | `string` | Free text       | No       | `""`                   | Optional human-readable description; stored in result JSON. |
| `steps`       | `array`  | See below       | Yes      | —                      | Ordered list of `AssemblyStep` objects. |

#### Field Reference — AssemblyStep

| Field               | Type                   | Unit / Format                              | Required | Default                | Description |
|---------------------|------------------------|--------------------------------------------|----------|------------------------|-------------|
| `step_id`           | `int`                  | Positive integer, unique within sequence   | Yes      | —                      | Step identifier; referenced by `prerequisites`. |
| `component_id`      | `int`                  | Must match an `id` in `components.json`    | Yes      | —                      | Selects which component is moved in this step. |
| `target_position`   | `[float, float, float]`| Meters (m), `[x, y, z]` in world frame    | Yes      | —                      | Final resting position of the component after this step. |
| `target_orientation`| `[float, float, float, float]` | Quaternion `[x, y, z, w]`        | No       | `[0.0, 0.0, 0.0, 1.0]` | Final orientation; same axis-angle conversion as components applies. |
| `prerequisites`     | `[int, ...]`           | List of `step_id` values                   | No       | `[]`                   | Steps that must be completed before this step can begin; used to build the dependency graph. |
| `tools_used`        | `[string, ...]`        | List of tool-name strings                  | No       | `[]`                   | Tools actively used during this step; merged with `tools_required` from the component for conflict analysis. |

---

## Configurable Parameters

The following constants and constructor arguments can be adjusted directly in the source code to tune simulation behavior. No external configuration file is required.

| Location in code | Parameter | Default | Effect |
|---|---|---|---|
| `PyBulletSimulator.__init__` | `gui_mode: bool` | `True` | Switches between GUI mode (interactive window) and headless DIRECT mode. |
| `PyBulletSimulator.__init__` | `gravity: float` | `-9.81` | Gravitational acceleration in m/s² along the Z-axis. |
| `PyBulletSimulator._initialize_simulation` | `p.setTimeStep(1.0 / 240.0)` | `1/240 s` | Physics time step; finer steps increase accuracy but extend runtime. |
| `PyBulletSimulator.load_component` (changeDynamics call) | `lateralFriction` | `1.5` | Coulomb friction coefficient between component and other bodies. |
| `PyBulletSimulator.load_component` (changeDynamics call) | `contactStiffness` / `contactDamping` | `1000` / `100` | Contact spring-damper parameters controlling penetration response. |
| `PyBulletSimulator.load_component` (changeDynamics call) | `linearDamping` / `angularDamping` | `2` / `3` | Velocity damping for free-floating objects; reduce settling time. |
| `PyBulletSimulator.change_dynamics_all_objects` | All dynamics parameters | Higher values post-load | Applied globally after all components are loaded; stiffens contacts for the actual movement phase. |
| `PyBulletSimulator.fix_object_after_assembly` | `linearDamping` / `angularDamping` | `8.0` / `8.0` | High damping values applied after a step finishes to freeze placed parts. |
| `PyBulletSimulator.simulate_assembly_step` | `duration: float` | `2.0 s` | Wall-clock simulation duration per step; scales the number of physics iterations. |
| `PyBulletSimulator.simulate_assembly_step` | `liftheight` | `0.05 m` | Vertical clearance added to waypoints to lift the component above obstacles before lateral movement. |
| `PyBulletSimulator._compute_min_clearance` | `search_distance: float` | `0.2 m` | Maximum distance at which closest-point queries are evaluated; limits the clearance search radius. |
| `PyBulletSimulator._check_tool_access_for_pose` | `tool_length: float` | `0.12 m` | Length of the simulated tool used in ray-cast accessibility checks. |
| `PyBulletSimulator._check_tool_access_for_pose` | `radial_offset: float` | `0.02 m` | Lateral offset of the ray-cast origin to simulate a tool approaching at a slight angle. |
| `TimeEfficiencyCriterion.__init__` | `gamma: float` | `0.5` | Weighting factor that balances parallelism degree against tool-resource conflicts (range [0, 1]). |
| `AccessibilityCriterion.calculate` | Clearance thresholds `c_min`, `c1`, `c2`, `c_max` | `0, 10, 40, 80 mm` | Trapezoidal membership function boundaries for the fuzzy clearance index. |
| `StabilityCriterion.__init__` | `quantitative_weights` / `qualitative_weights` | See code | TOPSIS indicator weights (must sum to 1.0 across all five indicators combined). |
| `QualityEvaluator._initialize_default_criteria` | `weight` per criterion | `0.25` each | Relative weight of each criterion in the final composite score. |
| `AssemblyFramework.__init__` | `self.output_dir` | `"assembly_results"` | Directory where all output files are written. |

---

## Quality Criteria

All criteria return a normalised score in the interval **[0.0, 1.0]** where higher values indicate better assembly quality. The final composite score is the weighted arithmetic mean of all active criteria, computed in `QualityEvaluator.evaluate_sequence`.

### Adding New Criteria

A new quality criterion must fulfill the following contract:

1. **Inherit from `QualityCriterion`** (abstract base class in the source file).
2. **Implement three methods:**
   - `calculate(sequence, components, simulation_data) -> float` — returns a score in [0.0, 1.0].
   - `get_name() -> str` — returns a unique string identifier used as the dictionary key in all output files.
   - `get_description() -> str` — returns a human-readable description stored in log output.
3. **Register the criterion** in `QualityEvaluator._initialize_default_criteria` using `self.add_criterion(YourCriterion(), weight=<float>)`. The weight is a relative value; the evaluator normalises by the total weight sum, so absolute magnitude does not matter, only the ratio between criteria weights.

The `simulation_data` dictionary passed to `calculate` contains all step-level results produced by `PyBulletSimulator.simulate_assembly_step`, including per-step collision lists, minimum clearance values (`clearance_step_<id>`), tool-access flags (`tool_access_step_<id>`), the global collision count, and the assembly center of mass.

```python
class MyCriterion(QualityCriterion):
    def calculate(self, sequence, components, simulation_data):
        # your logic here
        return max(0.0, min(1.0, score))

    def get_name(self):
        return "My_Criterion"

    def get_description(self):
        return "Brief explanation of what this criterion measures."
```

---

### 1. Time Efficiency (Zeiteffizienz)

**Class:** `TimeEfficiencyCriterion` — based on Zhao et al. (2012).

The criterion measures how efficiently the assembly exploits parallelism and avoids tool-resource conflicts. It combines two sub-scores:

**Parallelism degree** \(d_\text{para}\) is derived from the dependency graph. The actual depth \(t_i\) of the prerequisite tree is compared against the theoretical bounds:

\[
d_\text{para} = \frac{t_\text{max} - t_i}{t_\text{max} - t_\text{min}}, \quad t_\text{max} = n, \quad t_\text{min} = \lceil \log_2 n \rceil
\]

where \(n\) is the total number of steps. A fully sequential sequence reaches \(d_\text{para} = 0\); the most parallelisable structure reaches \(d_\text{para} = 1\).

**Tool conflict ratio** counts pairs of steps that share at least one tool and have no prerequisite relationship (i.e., could be executed concurrently). The ratio of conflicting pairs to all potential parallel pairs is subtracted from a resource factor.

The final score is:

\[
\text{TimeEfficiency} = \gamma \cdot d_\text{para} + (1 - \gamma)(1 - \text{tool\_conflict\_ratio})
\]

The parameter \(\gamma\) (default 0.5) controls the relative importance of parallelism versus resource availability.

---

### 2. Stability (Stabilität)

**Class:** `StabilityCriterion` — based on Ma et al. (2015), using a TOPSIS decision method.

The criterion collects five weighted indicators across two categories:

**Quantitative indicators** (computed analytically):

| Indicator | Weight | Computation |
|---|---|---|
| `gravity_direction_count` | 0.237 | Number of steps where the target position vector is more than 70 % aligned with the Z-axis (gravity direction). |
| `positioning_bases` | 0.186 | Sum of `connection_points` across all components. |
| `assembly_relationship` | 0.200 | Number of established contacts reported by the simulation, falling back to the step count. |

**Qualitative indicators** (fuzzy-evaluated via a trapezoidal membership approximation):

| Indicator | Weight | Computation |
|---|---|---|
| `contact_type_quality` | 0.130 | Per-step connection score \(\min(1, \text{connection\_points}/5)\), smoothed by a symmetric fuzzy membership \((a + 2b + c)/4\). |
| `structural_stability` | 0.248 | Weighted combination of three sub-factors: center-of-mass height (40 %), collision count penalty (35 %), and joint stress estimate (25 %), each smoothed by the same fuzzy membership. |

After normalisation, each indicator is weighted and a TOPSIS score is computed. Because the positive ideal solution equals the observed best value (all benefit-type indicators), the positive ideal distance is zero, and the TOPSIS score collapses to:

\[
\text{Stability} = \frac{d^-}{d^- + d^+} = \frac{d^-}{d^-} = 1 \cdot \text{(relative distance from negative ideal)}
\]

The score reflects how far the sequence is from the worst possible configuration.

---

### 3. Accessibility (Zugänglichkeit)

**Class:** `AccessibilityCriterion`

The criterion combines three independent indices, each averaged over all steps:

**Space accessibility index** \(g_\text{space}\) penalises collision events detected during each step:

\[
g_\text{space} = 1 - \frac{k}{l + k}
\]

where \(k\) is the total number of collision events across all steps and \(l\) is the number of collision-free steps.

**Clearance index** \(\mu_c\) applies a trapezoidal fuzzy membership function to the minimum measured clearance \(c\) (in mm) per step:

\[
\mu_c = \begin{cases} 0 & c \leq 0 \text{ or } c \geq 80 \\ \frac{c}{10} & 0 < c < 10 \\ 1 & 10 \leq c \leq 40 \\ \frac{80 - c}{40} & 40 < c < 80 \end{cases}
\]

The clearance is measured by `_compute_min_clearance` via PyBullet's `getClosestPoints` API at each simulation tick.

**Tool access index** \(\mu_t\) is determined by `_check_tool_access_for_pose`, which fires ray-casts in five approach directions (+X, −X, +Y, −Y, +Z). If at least one direction is unobstructed, the step is considered accessible (\(\mu_t = 1.0\)); otherwise \(\mu_t = 0.2\). Unverified steps default to \(\mu_t = 0.5\).

The final score is the arithmetic mean of all three indices:

\[
\text{Accessibility} = \frac{g_\text{space} + \bar{\mu}_c + \bar{\mu}_t}{3}
\]

---

### 4. Complexity (Komplexität)

**Class:** `ComplexityCriterion` — based on Assembly Sequence Flexibility (ASF).

The criterion measures the structural constraint density of the sequence. It builds the transitive closure of the prerequisite graph (Floyd–Warshall) to determine all pairs \((i, j)\) with a strict ordering relation. The **ordering share** \(OS\) is:

\[
OS = \frac{p_\text{pr}}{p_\text{all}}, \quad p_\text{all} = \frac{n(n-1)}{2}
\]

where \(p_\text{pr}\) counts step pairs with a definite precedence relation. A high \(OS\) means the sequence is heavily constrained (low flexibility); the score inverts this:

\[
\text{Complexity} = 1 - OS
\]

A fully unconstrained sequence (no prerequisites) yields a score of 1.0; a fully linear chain yields a score approaching 0.0.

---

## Execution Modes

The simulation can run in two modes, controlled by the `gui_mode` parameter passed to `AssemblyFramework` (which forwards it to `PyBulletSimulator`).

### GUI Mode (`gui_mode=True`, default)

PyBullet opens an interactive 3-D viewer window. Each assembly step is rendered in real time, with movement interpolated at a visual rate of 480 ticks per second (`time.sleep(1/480)` per tick). The window remains open after the simulation completes until the user presses Enter. This mode is intended for visual inspection and debugging.

### Headless Mode (`gui_mode=False`)

PyBullet connects in `DIRECT` mode without any graphical output. All `time.sleep` calls are bypassed, which significantly reduces wall-clock runtime. This mode is recommended for batch evaluations, CI pipelines, or server environments without a display.

### Fallback Mode (Example Data)

If either `components.json` or `sequence.json` is not found, the `main` function catches the `FileNotFoundError` and substitutes built-in example data (`create_example_components` and `create_example_sequence`). The simulation then proceeds normally in GUI mode with these defaults, which is useful for verifying the installation without preparing any input files.

---

## Output Files

All output files are written to the `assembly_results/` directory (configurable via `self.output_dir`).

### 1. JSON Result File

**Filename pattern:** `assembly_results_<sequence_name>_<YYYYMMDD_HHMMSS>.json`

One file is created per evaluated sequence. The file contains the full simulation record:

```
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
    "center_of_mass": [x, y, z],   // meters
    "total_mass": float,            // kg
    "total_duration": float,        // seconds
    "steps": [
      {
        "step_id": int,
        "component_id": int,
        "collisions": [{"time": float, "contactcount": int}, ...],
        "positions": [[x, y, z], ...],
        "orientations": [[x, y, z, w], ...],
        "min_clearance": float,         // meters
        "tool_access_free": bool,
        "duration": float,              // seconds
        "clearance_step_<id>": float,   // meters
        "tool_access_step_<id>": bool
      }, ...
    ]
  },
  "evaluation_results": {
    "Zeiteffizienz": float,     // [0.0, 1.0]
    "Stabilität": float,        // [0.0, 1.0]
    "Zugänglichkeit": float,    // [0.0, 1.0]
    "Komplexität": float,       // [0.0, 1.0]
    "Gesamtbewertung": float    // weighted mean, [0.0, 1.0]
  },
  "timestamp": ISO-8601 string
}
```

### 2. CSV Summary File

**Filename pattern:** `assembly_evaluation_results_<YYYYMMDD_HHMMSS>.csv`

Produced by `export_results_to_csv`. One row per evaluated sequence; columns are fixed metadata fields followed by one `Score_<name>` column per registered criterion:

| Column | Content |
|---|---|
| `Sequence_ID` | Integer sequence identifier |
| `Sequence_Name` | String name of the sequence |
| `Step_Count` | Number of assembly steps |
| `Total_Duration` | Cumulative simulation duration in seconds |
| `Collision_Count` | Total contact-point events across all steps |
| `Total_Mass` | Sum of all component masses in kg |
| `Timestamp` | ISO-8601 evaluation timestamp |
| `Score_<criterion>` | Float score [0.0, 1.0] for each registered criterion |

### 3. Log File

**Filename:** `assembly_simulation.log` (written to the working directory)

Plain-text log in the format `TIMESTAMP - LOGGER - LEVEL - MESSAGE`. Records loading errors, fallback geometry activations, and any exceptions raised during step simulation. Simultaneously echoed to the terminal (`StreamHandler`).

---

## Command-Line Output

When running `python assembly_simulation_framework.py`, the terminal displays the following information in sequence:

```
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

The JSON block is a pretty-printed dump of `evaluation_results`. All intermediate per-step details (positions, orientations, clearance samples) are not printed to the terminal but are fully captured in the JSON output file. Warning and error messages from the logger appear inline between these blocks if geometry loading fails or a component ID is missing from the sequence.
