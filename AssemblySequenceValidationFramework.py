# -*- coding: utf-8 -*-
"""
Montagereihenfolge Validierungsframework für PyBullet
=====================================================

Entwicklungsumgebung zur Simulation und Bewertung von Montagereihenfolgen
basierend auf literaturbasierten Gütekriterien für Masterarbeitsforschung.

Dieses Framework implementiert die Anforderungen aus der Masterarbeit
"Validierung der Montagereihenfolge" und stellt eine virtuelle Umgebung
zur Erprobung verschiedener Montageprozesse bereit.

Hauptfunktionalitäten:
- Laden und Simulation von Bauteilen in PyBullet
- Implementierung verschiedener Gütekriterien aus der Literatur
- Bewertung von Montagereihenfolgen
- Datenerfassung für wissenschaftliche Auswertung
- Visualisierung und Protokollierung der Ergebnisse
"""

import pybullet as p
import pybullet_data
import numpy as np
import pandas as pd
import json
import time
import math
import logging
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from datetime import datetime
import os
import csv

# Konfiguration des Logging-Systems
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("assembly_simulation.log"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Datenklassen
# ---------------------------------------------------------------------------


@dataclass
class Component:
    """
    Datenklasse für Bauteile in der Montagesimulation.

    Attributes:
        id: Eindeutige Identifikation des Bauteils
        name: Bezeichnung des Bauteils
        urdf_path: Pfad zur URDF- oder STL-Datei
        mass: Masse des Bauteils in kg
        position: Startposition [x, y, z]
        orientation: Startorientierung [x, y, z, w] (Quaternion)
        dimensions: Abmessungen [length, width, height]
        connection_points: Anzahl Verbindungspunkte zu anderen Bauteilen
        tools_required: Benötigte Werkzeuge für die Montage
    """

    id: int
    name: str
    urdf_path: str
    mass: float
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    dimensions: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    connection_points: int = 0
    tools_required: List[str] = field(default_factory=list)
    physics_id: Optional[int] = None


@dataclass
class AssemblyStep:
    """
    Datenklasse für einzelne Montageschritte.

    Attributes:
        step_id: Eindeutige Schritt-ID
        component_id: ID des zu montierenden Bauteils
        target_position: Zielposition nach Montage
        target_orientation: Zielorientierung nach Montage
        prerequisites: Voraussetzungen (andere Schritte)
        tools_used: Verwendete Werkzeuge
    """

    step_id: int
    component_id: int
    target_position: List[float]
    target_orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    prerequisites: List[int] = field(default_factory=list)
    tools_used: List[str] = field(default_factory=list)


@dataclass
class AssemblySequence:
    """
    Datenklasse für eine komplette Montagereihenfolge.

    Attributes:
        sequence_id: Eindeutige Sequenz-ID
        name: Bezeichnung der Sequenz
        steps: Liste der Montageschritte
        total_time: Gesamtzeit der Montage
        description: Beschreibung der Sequenz
    """

    sequence_id: int
    name: str
    steps: List[AssemblyStep]
    total_time: float = 0.0
    description: str = ""


# ---------------------------------------------------------------------------
# Abstrakte Basisklasse für Gütekriterien
# ---------------------------------------------------------------------------


class QualityCriterion(ABC):
    """
    Abstrakte Basisklasse für Gütekriterien zur Bewertung von Montagereihenfolgen.

    Diese Klasse definiert die Schnittstelle für alle Gütekriterien, die in der
    Literatur zur Bewertung von Montagereihenfolgen verwendet werden.
    """

    @abstractmethod
    def calculate(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> float:
        """
        Berechnet das Gütekriterium für eine gegebene Montagereihenfolge.

        Args:
            sequence: Die zu bewertende Montagereihenfolge
            components: Liste aller Bauteile
            simulation_data: Simulationsdaten aus PyBullet

        Returns:
            Bewertungswert im Bereich [0.0, 1.0]
        """
        raise NotImplementedError

    @abstractmethod
    def get_name(self) -> str:
        """Gibt den Namen des Gütekriteriums zurück."""
        raise NotImplementedError

    @abstractmethod
    def get_description(self) -> str:
        """Gibt eine Beschreibung des Gütekriteriums zurück."""
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Zeiteffizienz nach Zhao et al.
# ---------------------------------------------------------------------------


class TimeEfficiencyCriterion(QualityCriterion):
    """
    Gütekriterium für Zeiteffizienz der Montagereihenfolge nach Zhao et al.

    Bewertet die Montagereihenfolge basierend auf der Montageparallelität.
    Die Montageparallelität ergibt sich aus der Tiefe des Montagebaums und
    Werkzeugkonflikten zwischen parallelisierbaren Schritten.
    """

    def __init__(self, gamma: float = 0.5) -> None:
        """
        Initialisiert das Kriterium.

        Args:
            gamma: Gewichtung der Werkzeugkonflikte im Bereich [0, 1]
        """
        self.gamma = gamma

    def calculate(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> float:
        n_parts = len(sequence.steps)
        if n_parts <= 1:
            return 1.0

        # 1) Baumtiefe bestimmen
        t_i = self._calculate_tree_depth(sequence)
        t_max = n_parts
        t_min = math.ceil(math.log2(n_parts)) if n_parts > 1 else 1

        if t_max == t_min:
            d_para = 1.0
        else:
            d_para = (t_max - t_i) / (t_max - t_min)

        d_para = max(0.0, min(1.0, d_para))

        # 2) Werkzeugkonflikte
        tool_conflict_ratio = self._compute_tool_conflicts(sequence, components)

        # 3) Effektive Parallelität
        resource_factor = 1.0 - self.gamma * tool_conflict_ratio
        time_efficiency = d_para * resource_factor

        logger.info(
            "Zeiteffizienz (Zhao+Tools): D_para=%.3f, ToolConflictRatio=%.3f, "
            "gamma=%.2f, Score=%.3f (Tiefe: %d, Min: %d, Max: %d)",
            d_para,
            tool_conflict_ratio,
            self.gamma,
            time_efficiency,
            t_i,
            t_min,
            t_max,
        )

        return max(0.0, min(1.0, time_efficiency))

    def _calculate_tree_depth(self, sequence: AssemblySequence) -> int:
        step_depths: Dict[int, int] = {}
        sorted_steps = sorted(sequence.steps, key=lambda s: s.step_id)

        for step in sorted_steps:
            if not step.prerequisites:
                step_depths[step.step_id] = 1
            else:
                prerequisite_depths: List[int] = []
                for prereq_id in step.prerequisites:
                    if prereq_id in step_depths:
                        prerequisite_depths.append(step_depths[prereq_id])
                    else:
                        prerequisite_depths.append(1)
                step_depths[step.step_id] = (
                    max(prerequisite_depths) + 1 if prerequisite_depths else 1
                )

        max_depth = max(step_depths.values()) if step_depths else 1
        return max_depth

    def _compute_tool_conflicts(
        self, sequence: AssemblySequence, components: List[Component]
    ) -> float:
        comp_tools: Dict[int, set] = {
            c.id: set(c.tools_required or []) for c in components
        }

        step_tools: List[set] = []
        for step in sequence.steps:
            comp_tools_used = comp_tools.get(step.component_id, set())
            step_tools_used = set(step.tools_used or [])
            step_tools.append(step_tools_used | comp_tools_used)

        conflicts = 0
        n_parallel_checks = 0

        for i in range(len(sequence.steps)):
            for j in range(i + 1, len(sequence.steps)):
                step_i = sequence.steps[i]
                step_j = sequence.steps[j]

                if (
                    step_j.step_id not in step_i.prerequisites
                    and step_i.step_id not in step_j.prerequisites
                ):
                    n_parallel_checks += 1
                    if step_tools[i] & step_tools[j]:
                        conflicts += 1

        if n_parallel_checks == 0:
            return 0.0

        conflict_ratio = conflicts / n_parallel_checks
        return max(0.0, min(1.0, conflict_ratio))

    def get_name(self) -> str:
        return "Zeiteffizienz"

    def get_description(self) -> str:
        return (
            "Bewertet die Zeiteffizienz basierend auf der Montageparallelität "
            "nach Zhao et al. (2012)."
        )


# ---------------------------------------------------------------------------
# Stabilität nach Ma et al. (TOPSIS)
# ---------------------------------------------------------------------------


class StabilityCriterion(QualityCriterion):
    """
    Gütekriterium für Stabilität während der Montage nach Ma et al. (2015).

    Implementiert ein vereinfachtes TOPSIS-Framework mit quantitativen und
    qualitativen Indikatoren.
    """

    def __init__(self) -> None:
        self.quantitative_weights = {
            "gravity_direction_count": 0.167,
            "positioning_bases": 0.131,
            "assembly_relationship": 0.141,
        }
        self.qualitative_weights = {
            "contact_type_quality": 0.092,
            "structural_stability": 0.175,
        }
        self.overall_weight = 0.092
        logger.info(
            "TOPSIS-basiertes Stabilitätskriterium nach Ma et al. initialisiert"
        )

    def calculate(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> float:
        quantitative = self._collect_quantitative_indicators(
            sequence, components, simulation_data
        )
        qualitative = self._collect_qualitative_indicators(
            sequence, components, simulation_data
        )

        norm_quant = self._normalize_indicators(quantitative, "quantitative")
        norm_qual = self._normalize_indicators(qualitative, "qualitative")

        weighted = self._apply_weights(norm_quant, norm_qual)
        stability_score = self._calculate_topsis_score(weighted)

        logger.info("Stabilität berechnet (TOPSIS): %.3f", stability_score)
        return max(0.0, min(1.0, stability_score))

    def _collect_quantitative_indicators(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> Dict[str, float]:
        indicators: Dict[str, float] = {}

        gravity_aligned_count = 0
        for step in sequence.steps:
            target_pos = step.target_position
            if len(target_pos) >= 3:
                z_component = abs(target_pos[2])
                total_magnitude = math.sqrt(sum(v ** 2 for v in target_pos))
                if total_magnitude > 0:
                    z_ratio = z_component / total_magnitude
                    if z_ratio > 0.7:
                        gravity_aligned_count += 1
        indicators["gravity_direction_count"] = gravity_aligned_count

        positioning_bases_count = 0
        for component in components:
            positioning_bases_count += component.connection_points
        indicators["positioning_bases"] = positioning_bases_count

        if "established_contacts" in simulation_data:
            assembly_relationships = simulation_data["established_contacts"]
        else:
            assembly_relationships = len(sequence.steps)
        indicators["assembly_relationship"] = assembly_relationships

        return indicators

    def _collect_qualitative_indicators(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> Dict[str, float]:
        indicators: Dict[str, float] = {}

        contact_quality_scores: List[float] = []
        for step in sequence.steps:
            component = next((c for c in components if c.id == step.component_id), None)
            if component is None:
                continue
            connection_score = min(1.0, component.connection_points / 5.0)
            b = connection_score
            a = max(0.0, b - 0.1)
            c_val = min(1.0, b + 0.1)
            score = (a + 2 * b + c_val) / 4.0
            contact_quality_scores.append(score)

        indicators["contact_type_quality"] = (
            sum(contact_quality_scores) / len(contact_quality_scores)
            if contact_quality_scores
            else 0.5
        )

        stability_factors: List[float] = []

        center_of_mass = simulation_data.get("center_of_mass", [0.0, 0.0, 0.5])
        com_height = center_of_mass[2] if len(center_of_mass) > 2 else 0.5
        com_score = max(0.0, 1.0 - min(1.0, com_height / 1.0))
        b_com = com_score
        a_com = max(0.0, b_com - 0.15)
        c_com = min(1.0, b_com + 0.05)
        stability_factors.append((a_com + 2 * b_com + c_com) / 4.0)

        collision_count = simulation_data.get("collision_count", 0)
        collision_score = max(0.0, 1.0 - min(1.0, collision_count / 10.0))
        b_col = collision_score
        a_col = max(0.0, b_col - 0.1)
        c_col = min(1.0, b_col + 0.1)
        stability_factors.append((a_col + 2 * b_col + c_col) / 4.0)

        max_joint_stress = simulation_data.get("max_joint_stress", 50.0)
        stress_score = max(0.0, 1.0 - min(1.0, max_joint_stress / 200.0))
        b_str = stress_score
        a_str = max(0.0, b_str - 0.1)
        c_str = min(1.0, b_str + 0.1)
        stability_factors.append((a_str + 2 * b_str + c_str) / 4.0)

        indicators["structural_stability"] = (
            stability_factors[0] * 0.40
            + stability_factors[1] * 0.35
            + stability_factors[2] * 0.25
        )

        return indicators

    def _normalize_indicators(
        self, indicators: Dict[str, float], indicator_type: str
    ) -> Dict[str, float]:
        normalized: Dict[str, float] = {}
        if not indicators:
            return normalized

        benefit_indicators = set(indicators.keys())
        values: List[float] = []

        for key, value in indicators.items():
            if value == 0:
                normalized[key] = 0.0
            else:
                if key in benefit_indicators:
                    max_value = max(indicators.values()) if indicators.values() else 1.0
                    normalized[key] = value / max_value if max_value > 0 else 0.0
                else:
                    min_value = min(indicators.values()) if indicators.values() else 1.0
                    normalized[key] = min_value / value if value > 0 else 0.0
            values.append(normalized[key])

        vector_magnitude = math.sqrt(sum(v ** 2 for v in values))
        if vector_magnitude > 0:
            for key in normalized:
                normalized[key] /= vector_magnitude

        return normalized

    def _apply_weights(
        self, quantitative: Dict[str, float], qualitative: Dict[str, float]
    ) -> Dict[str, float]:
        weighted: Dict[str, float] = {}

        for key, value in quantitative.items():
            weight = self.quantitative_weights.get(key, 0.0)
            weighted[key] = value * weight

        for key, value in qualitative.items():
            weight = self.qualitative_weights.get(key, 0.0)
            weighted[key] = value * weight

        return weighted

    def _calculate_topsis_score(self, weighted_indicators: Dict[str, float]) -> float:
        if not weighted_indicators:
            return 0.5

        z_positive: Dict[str, float] = {}
        z_negative: Dict[str, float] = {}

        for key, value in weighted_indicators.items():
            z_positive[key] = value
            z_negative[key] = 0.0

        distance_positive = 0.0
        distance_negative = 0.0

        for key, value in weighted_indicators.items():
            distance_positive += (value - z_positive[key]) ** 2
            distance_negative += (value - z_negative[key]) ** 2

        distance_positive = math.sqrt(distance_positive)
        distance_negative = math.sqrt(distance_negative)

        denominator = distance_positive + distance_negative
        if denominator == 0.0:
            return 1.0
        return distance_negative / denominator

    def get_name(self) -> str:
        return "Stabilität"

    def get_description(self) -> str:
        return (
            "Bewertet die strukturelle Stabilität während der Montage nach Ma et al. "
            "(2015) mit einem TOPSIS-Ansatz."
        )


# ---------------------------------------------------------------------------
# Zugänglichkeit nach Liu et al.
# ---------------------------------------------------------------------------


class AccessibilityCriterion(QualityCriterion):
    """
    Gütekriterium für Zugänglichkeit der Bauteile nach Liu et al. (2024).
    """

    def calculate(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> float:
        steps_data = simulation_data.get("steps", [])

        total_interference_events = 0
        free_direction_steps = 0

        clearance_scores: List[float] = []
        tool_scores: List[float] = []

        for step in sequence.steps:
            step_data = next(
                (sd for sd in steps_data if sd.get("step_id") == step.step_id), {}
            )

            collisions = step_data.get("collisions", [])
            k_step = len(collisions)
            total_interference_events += k_step
            if k_step == 0:
                free_direction_steps += 1

            clearance = simulation_data.get(f"clearance_step_{step.step_id}", None)
            if clearance is not None:
                c_min = 0.0
                c1 = 10.0
                c2 = 40.0
                c_max = 80.0
                if clearance <= c_min or clearance >= c_max:
                    mu_c = 0.0
                elif c1 <= clearance <= c2:
                    mu_c = 1.0
                elif c_min < clearance < c1:
                    mu_c = (clearance - c_min) / (c1 - c_min)
                else:
                    mu_c = (c_max - clearance) / (c_max - c2)
                clearance_scores.append(mu_c)

            tool_access = simulation_data.get(f"tool_access_step_{step.step_id}", None)
            if tool_access is True:
                mu_t = 1.0
            elif tool_access is False:
                mu_t = 0.2
            else:
                mu_t = 0.5
            tool_scores.append(mu_t)

        k = total_interference_events
        l = free_direction_steps

        if (l + k) == 0:
            g_space = 1.0
        else:
            g_space = 1.0 - (k / (l + k))

        clearance_index = (
            sum(clearance_scores) / len(clearance_scores)
            if clearance_scores
            else 1.0
        )
        tool_index = sum(tool_scores) / len(tool_scores) if tool_scores else 1.0

        accessibility_score = (g_space + clearance_index + tool_index) / 3.0

        logger.info(
            "Accessibility berechnet: Score=%.3f, g_space=%.3f, "
            "clearance_index=%.3f, tool_index=%.3f, k=%d, l=%d",
            accessibility_score,
            g_space,
            clearance_index,
            tool_index,
            k,
            l,
        )

        return max(0.0, min(1.0, accessibility_score))

    def get_name(self) -> str:
        return "Zugänglichkeit"

    def get_description(self) -> str:
        return (
            "Bewertet die Zugänglichkeit der Bauteile auf Basis der Assembly Space "
            "Operability nach Liu et al. (2024) sowie fuzzy-bewertetem Freiraum und "
            "Werkzeugzugang."
        )


# ---------------------------------------------------------------------------
# Komplexität nach Bleckmann et al.
# ---------------------------------------------------------------------------


class ComplexityCriterion(QualityCriterion):
    """
    Gütekriterium für strukturelle Komplexität der Montagereihenfolge
    auf Basis der Assembly Sequence Flexibility (ASF) nach Bleckmann et al.
    """

    def calculate(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> float:
        steps = sequence.steps
        n = len(steps)
        if n <= 1:
            return 1.0

        step_ids = [step.step_id for step in steps]
        id_to_index = {sid: idx for idx, sid in enumerate(step_ids)}

        precedes = [[False] * n for _ in range(n)]
        for step in steps:
            j = id_to_index[step.step_id]
            for pre_id in step.prerequisites:
                if pre_id in id_to_index:
                    i = id_to_index[pre_id]
                    precedes[i][j] = True

        for k in range(n):
            for i in range(n):
                if precedes[i][k]:
                    for j in range(n):
                        if precedes[k][j]:
                            precedes[i][j] = True

        P_pr = 0
        P_all = n * (n - 1) // 2
        if P_all == 0:
            return 1.0

        for i in range(n):
            for j in range(i + 1, n):
                if precedes[i][j] and not precedes[j][i]:
                    pr_ij = 1
                elif precedes[j][i] and not precedes[i][j]:
                    pr_ij = -1
                else:
                    pr_ij = 0

                if pr_ij != 0:
                    P_pr += 1

        OS = P_pr / P_all
        ASF = 1.0 - OS
        ASF = max(0.0, min(1.0, ASF))

        logger.info(
            "Komplexität (ASF-basiert) berechnet: ASF=%.3f, OS=%.3f, P_pr=%d, P_all=%d, n=%d",
            ASF,
            OS,
            P_pr,
            P_all,
            n,
        )
        return ASF

    def get_name(self) -> str:
        return "Komplexität"

    def get_description(self) -> str:
        return (
            "Bewertet die strukturelle Komplexität der Montagereihenfolge über die "
            "Assembly Sequence Flexibility (ASF) nach Bleckmann et al."
        )


# ---------------------------------------------------------------------------
# QualityEvaluator
# ---------------------------------------------------------------------------


class QualityEvaluator:
    """
    Zentrale Klasse zur Bewertung von Montagereihenfolgen.

    Verwaltet verschiedene Gütekriterien und führt Gesamtbewertungen durch.
    """

    def __init__(self) -> None:
        self.criteria: List[QualityCriterion] = []
        self.weights: Dict[str, float] = {}
        self._initialize_default_criteria()

    def _initialize_default_criteria(self) -> None:
        default_criteria: List[QualityCriterion] = [
            TimeEfficiencyCriterion(),
            StabilityCriterion(),
            AccessibilityCriterion(),
            ComplexityCriterion(),
        ]
        for criterion in default_criteria:
            self.add_criterion(criterion, weight=0.25)

    def add_criterion(self, criterion: QualityCriterion, weight: float = 1.0) -> None:
        self.criteria.append(criterion)
        self.weights[criterion.get_name()] = weight
        logger.info(
            "Gütekriterium '%s' hinzugefügt (Gewicht: %.3f)",
            criterion.get_name(),
            weight,
        )

    def evaluate_sequence(
        self,
        sequence: AssemblySequence,
        components: List[Component],
        simulation_data: Dict,
    ) -> Dict[str, float]:
        results: Dict[str, float] = {}
        total_weighted_score = 0.0
        total_weight = 0.0

        for criterion in self.criteria:
            score = criterion.calculate(sequence, components, simulation_data)
            name = criterion.get_name()
            results[name] = score
            weight = self.weights.get(name, 1.0)
            total_weighted_score += score * weight
            total_weight += weight

        overall_score = total_weighted_score / total_weight if total_weight > 0 else 0.0
        results["Gesamtbewertung"] = overall_score
        logger.info(
            "Sequenz '%s' bewertet. Gesamtbewertung: %.3f", sequence.name, overall_score
        )
        return results


# ---------------------------------------------------------------------------
# PyBullet-Simulator
# ---------------------------------------------------------------------------


class PyBulletSimulator:
    """
    PyBullet-basierte Simulationsumgebung für Montagereihenfolgen.
    """

    def __init__(self, gui_mode: bool = True, gravity: float = -9.81) -> None:
        self.gui_mode = gui_mode
        self.gravity = gravity
        self.physics_client: Optional[int] = None
        self.loaded_objects: Dict[int, int] = {}
        self.simulation_data: Dict[str, Any] = {}
        self._initialize_simulation()

    def _initialize_simulation(self) -> None:
        try:
            if self.gui_mode:
                self.physics_client = p.connect(p.GUI)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            else:
                self.physics_client = p.connect(p.DIRECT)

            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, self.gravity)
            p.setTimeStep(1.0 / 240.0)
            self.plane_id = p.loadURDF("plane.urdf")
            logger.info("PyBullet-Simulation erfolgreich initialisiert")
        except Exception as e:
            logger.error("Fehler bei der Initialisierung der Simulation: %s", e)
            raise

    def load_component(self, component: Component) -> int:
        """
        Lädt ein Bauteil in die Simulation und gibt die Objekt-ID zurück.
        """
        try:
            path = component.urdf_path

            if path.lower().endswith(".stl"):
                collision_shape = p.createCollisionShape(
                    p.GEOM_MESH,
                    fileName=path,
                    meshScale=component.dimensions,
                )
                visual_shape = p.createVisualShape(
                    p.GEOM_MESH,
                    fileName=path,
                    meshScale=[1.0, 1.0, 1.0],
                    rgbaColor=[0.5, 0.5, 0.8, 1.0],
                )
                object_id = p.createMultiBody(
                    baseMass=component.mass,
                    baseCollisionShapeIndex=collision_shape,
                    baseVisualShapeIndex=visual_shape,
                    basePosition=component.position,
                    baseOrientation=component.orientation,
                )
            else:
                object_id = p.loadURDF(
                    path,
                    basePosition=component.position,
                    baseOrientation=component.orientation,
                    useFixedBase=False,
                )

            component.physics_id = object_id
            self.loaded_objects[component.id] = object_id

            p.changeDynamics(
                object_id,
                -1,
                mass=component.mass,
                linearDamping=0.1,
                angularDamping=0.1,
            )

            logger.info(
                "Bauteil '%s' geladen (PyBullet-ID: %d)", component.name, object_id
            )
            return object_id

        except Exception as e:
            logger.error(
                "Fehler beim Laden des Bauteils '%s': %s", component.name, e
            )
            return self._create_fallback_object(component)

    def _create_fallback_object(self, component: Component) -> int:
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[d / 2.0 for d in component.dimensions],
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[d / 2.0 for d in component.dimensions],
            rgbaColor=[0.5, 0.5, 0.8, 1.0],
        )
        object_id = p.createMultiBody(
            baseMass=component.mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=component.position,
            baseOrientation=component.orientation,
        )
        component.physics_id = object_id
        self.loaded_objects[component.id] = object_id
        logger.warning("Fallback-Objekt für '%s' erstellt", component.name)
        return object_id

    def simulate_assembly_step(
        self, step: AssemblyStep, component: Component, duration: float = 5.0
    ) -> Dict:
        if component.physics_id is None:
            logger.error("Bauteil '%s' nicht in Simulation geladen", component.name)
            return {}

        object_id = component.physics_id
        step_data: Dict[str, Any] = {
            "step_id": step.step_id,
            "component_id": step.component_id,
            "start_time": time.time(),
            "collisions": [],
            "positions": [],
            "orientations": [],
        }

        start_pos, start_orn = p.getBasePositionAndOrientation(object_id)
        target_pos = step.target_position
        target_orn = step.target_orientation

        simulation_steps = int(duration * 240)

        for i in range(simulation_steps):
            t = i / simulation_steps
            current_pos = [
                start_pos[j] + t * (target_pos[j] - start_pos[j]) for j in range(3)
            ]
            current_orn = p.getQuaternionSlerp(start_orn, target_orn, t)

            p.resetBasePositionAndOrientation(object_id, current_pos, current_orn)
            p.stepSimulation()

            pos, orn = p.getBasePositionAndOrientation(object_id)
            step_data["positions"].append(pos)
            step_data["orientations"].append(orn)

            contact_points = p.getContactPoints(bodyA=object_id)
            if contact_points:
                step_data["collisions"].append(
                    {"time": i / 240.0, "contact_count": len(contact_points)}
                )

            if self.gui_mode:
                time.sleep(1.0 / 240.0)

        step_data["end_time"] = time.time()
        step_data["duration"] = step_data["end_time"] - step_data["start_time"]

        logger.info(
            "Montageschritt %d simuliert (%.2fs)",
            step.step_id,
            step_data["duration"],
        )
        return step_data

    def get_simulation_data(self) -> Dict:
        data: Dict[str, Any] = {
            "timestamp": datetime.now().isoformat(),
            "loaded_objects": len(self.loaded_objects),
            "collision_count": 0,
            "center_of_mass": [0.0, 0.0, 0.0],
            "total_mass": 0.0,
        }

        for obj_id in self.loaded_objects.values():
            contacts = p.getContactPoints(bodyA=obj_id)
            data["collision_count"] += len(contacts)

        total_mass = 0.0
        weighted_position = np.array([0.0, 0.0, 0.0])

        for obj_id in self.loaded_objects.values():
            dynamics_info = p.getDynamicsInfo(obj_id, -1)
            mass = float(dynamics_info[0])
            pos, _ = p.getBasePositionAndOrientation(obj_id)
            total_mass += mass
            weighted_position += np.array(pos) * mass

        if total_mass > 0.0:
            data["center_of_mass"] = (weighted_position / total_mass).tolist()
            data["total_mass"] = total_mass

        return data

    def reset_simulation(self) -> None:
        p.resetSimulation()
        self.loaded_objects.clear()
        self.simulation_data.clear()
        self.plane_id = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, self.gravity)
        logger.info("Simulation zurückgesetzt")

    def disconnect(self) -> None:
        if self.physics_client is not None:
            p.disconnect()
            logger.info("PyBullet-Verbindung beendet")


# ---------------------------------------------------------------------------
# AssemblyFramework
# ---------------------------------------------------------------------------


class AssemblyFramework:
    """
    Hauptframework für die Montagereihenfolge-Validierung.
    """

    def __init__(self, gui_mode: bool = True) -> None:
        self.simulator = PyBulletSimulator(gui_mode=gui_mode)
        self.evaluator = QualityEvaluator()
        self.components: List[Component] = []
        self.sequences: List[AssemblySequence] = []
        self.results: List[Dict[str, Any]] = []

        self.output_dir = "assembly_results"
        os.makedirs(self.output_dir, exist_ok=True)

        logger.info("AssemblyFramework initialisiert")

    def load_components_from_file(self, file_path: str) -> None:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.components.clear()
            for comp_data in data.get("components", []):
                component = Component(**comp_data)
                self.components.append(component)
                self.simulator.load_component(component)

            logger.info("%d Bauteile aus '%s' geladen", len(self.components), file_path)
        except Exception as e:
            logger.error("Fehler beim Laden der Bauteildefinitionen: %s", e)
            raise

    def load_sequence_from_file(self, file_path: str) -> None:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            steps: List[AssemblyStep] = []
            for step_data in data.get("steps", []):
                step = AssemblyStep(**step_data)
                steps.append(step)

            sequence = AssemblySequence(
                sequence_id=data.get("sequence_id", 1),
                name=data.get("name", "Unbenannte Sequenz"),
                steps=steps,
                description=data.get("description", ""),
            )

            self.sequences.append(sequence)
            logger.info("Montagereihenfolge '%s' geladen", sequence.name)
        except Exception as e:
            logger.error("Fehler beim Laden der Montagereihenfolge: %s", e)
            raise

    def simulate_sequence(self, sequence: AssemblySequence) -> Dict:
        logger.info("Starte Simulation der Sequenz '%s'", sequence.name)

        self.simulator.reset_simulation()
        for component in self.components:
            self.simulator.load_component(component)

        simulation_results: Dict[str, Any] = {
            "sequence_id": sequence.sequence_id,
            "sequence_name": sequence.name,
            "start_time": datetime.now().isoformat(),
            "steps": [],
            "total_duration": 0.0,
        }

        for step in sequence.steps:
            component = next((c for c in self.components if c.id == step.component_id), None)
            if component is None:
                logger.warning(
                    "Bauteil %d für Schritt %d nicht gefunden",
                    step.component_id,
                    step.step_id,
                )
                continue

            step_result = self.simulator.simulate_assembly_step(step, component)
            simulation_results["steps"].append(step_result)
            simulation_results["total_duration"] += step_result.get("duration", 0.0)

        final_data = self.simulator.get_simulation_data()
        simulation_results.update(final_data)
        simulation_results["end_time"] = datetime.now().isoformat()

        logger.info(
            "Simulation abgeschlossen (%.2fs)", simulation_results["total_duration"]
        )
        return simulation_results

    def evaluate_sequence(
        self, sequence: AssemblySequence, simulation_data: Dict
    ) -> Dict[str, float]:
        return self.evaluator.evaluate_sequence(sequence, self.components, simulation_data)

    def run_full_evaluation(self, sequence: AssemblySequence) -> Dict:
        simulation_data = self.simulate_sequence(sequence)
        evaluation_results = self.evaluate_sequence(sequence, simulation_data)

        full_results: Dict[str, Any] = {
            "sequence_info": {
                "id": sequence.sequence_id,
                "name": sequence.name,
                "description": sequence.description,
                "step_count": len(sequence.steps),
            },
            "simulation_data": simulation_data,
            "evaluation_results": evaluation_results,
            "timestamp": datetime.now().isoformat(),
        }

        self.results.append(full_results)
        self._save_results(full_results)
        return full_results

    def compare_sequences(self, sequences: List[AssemblySequence]) -> Dict:
        comparison_results: Dict[str, Any] = {
            "comparison_timestamp": datetime.now().isoformat(),
            "sequences": {},
            "ranking": [],
            "best_sequence": None,
        }

        sequence_scores: List[Tuple[str, float, int]] = []

        for sequence in sequences:
            results = self.run_full_evaluation(sequence)
            overall_score = results["evaluation_results"]["Gesamtbewertung"]

            comparison_results["sequences"][sequence.name] = results
            sequence_scores.append((sequence.name, overall_score, sequence.sequence_id))

        sequence_scores.sort(key=lambda x: x[1], reverse=True)
        comparison_results["ranking"] = [
            {"name": name, "score": score, "id": seq_id}
            for name, score, seq_id in sequence_scores
        ]

        if sequence_scores:
            comparison_results["best_sequence"] = sequence_scores[0][0]

        self._save_comparison_results(comparison_results)
        logger.info("Vergleich von %d Sequenzen abgeschlossen", len(sequences))
        return comparison_results

    def _save_results(self, results: Dict) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"assembly_results_{results['sequence_info']['name']}_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        logger.info("Ergebnisse gespeichert: %s", filepath)

    def _save_comparison_results(self, results: Dict) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sequence_comparison_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        logger.info("Vergleichsergebnisse gespeichert: %s", filepath)

    def export_results_to_csv(self, filename: Optional[str] = None) -> None:
        if not self.results:
            logger.warning("Keine Ergebnisse zum Exportieren vorhanden")
            return

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"assembly_evaluation_results_{timestamp}.csv"

        filepath = os.path.join(self.output_dir, filename)

        csv_data: List[Dict[str, Any]] = []
        for result in self.results:
            row: Dict[str, Any] = {
                "Sequence_ID": result["sequence_info"]["id"],
                "Sequence_Name": result["sequence_info"]["name"],
                "Step_Count": result["sequence_info"]["step_count"],
                "Total_Duration": result["simulation_data"].get("total_duration", 0.0),
                "Collision_Count": result["simulation_data"].get("collision_count", 0),
                "Total_Mass": result["simulation_data"].get("total_mass", 0.0),
                "Timestamp": result["timestamp"],
            }
            for criterion, score in result["evaluation_results"].items():
                row[f"Score_{criterion}"] = score
            csv_data.append(row)

        with open(filepath, "w", newline="", encoding="utf-8") as csvfile:
            if csv_data:
                fieldnames = list(csv_data[0].keys())
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(csv_data)
        logger.info("Ergebnisse nach CSV exportiert: %s", filepath)

    def generate_report(self, sequence: AssemblySequence, results: Dict) -> str:
        report_lines: List[str] = [
            "=" * 60,
            "MONTAGEREIHENFOLGE BEWERTUNGSBERICHT",
            "=" * 60,
            "",
            f"Sequenz: {sequence.name}",
            f"ID: {sequence.sequence_id}",
            f"Beschreibung: {sequence.description}",
            f"Anzahl Schritte: {len(sequence.steps)}",
            f"Bewertungszeitpunkt: {results.get('timestamp', 'Unbekannt')}",
            "",
            "SIMULATIONSERGEBNISSE:",
            "-" * 30,
            f"Gesamtdauer: {results['simulation_data'].get('total_duration', 0.0):.2f} Sekunden",
            f"Kollisionen: {results['simulation_data'].get('collision_count', 0)}",
            f"Gesamtmasse: {results['simulation_data'].get('total_mass', 0.0):.2f} kg",
            "",
            "GÜTEKRITERIEN BEWERTUNG:",
            "-" * 30,
        ]

        for criterion, score in results["evaluation_results"].items():
            if criterion != "Gesamtbewertung":
                percentage = score * 100.0
                rating = self._get_rating(score)
                report_lines.append(
                    f"{criterion:<20}: {percentage:6.1f}% ({rating})"
                )

        overall_score = results["evaluation_results"].get("Gesamtbewertung", 0.0)
        overall_percentage = overall_score * 100.0
        overall_rating = self._get_rating(overall_score)

        report_lines.extend(
            [
                "",
                "=" * 30,
                f"GESAMTBEWERTUNG: {overall_percentage:.1f}% ({overall_rating})",
                "=" * 30,
                "",
                "EMPFEHLUNGEN:",
                "-" * 15,
            ]
        )

        recommendations = self._generate_recommendations(results)
        report_lines.extend(recommendations)
        report_lines.append("=" * 60)
        return "\n".join(report_lines)

    def _get_rating(self, score: float) -> str:
        if score >= 0.9:
            return "Ausgezeichnet"
        if score >= 0.8:
            return "Sehr gut"
        if score >= 0.7:
            return "Gut"
        if score >= 0.6:
            return "Befriedigend"
        if score >= 0.5:
            return "Ausreichend"
        return "Verbesserungsbedürftig"

    def _generate_recommendations(self, results: Dict) -> List[str]:
        recommendations: List[str] = []
        evaluation = results["evaluation_results"]

        if evaluation.get("Zeiteffizienz", 1.0) < 0.7:
            recommendations.append(
                "• Zeiteffizienz: Prüfen Sie Werkzeugwechsel und Rüstzeiten"
            )

        if evaluation.get("Stabilität", 1.0) < 0.7:
            recommendations.append(
                "• Stabilität: Überprüfen Sie die Schwerpunktlage und Kollisionen"
            )

        if evaluation.get("Zugänglichkeit", 1.0) < 0.7:
            recommendations.append(
                "• Zugänglichkeit: Verbessern Sie den Zugang zu Montagepunkten"
            )

        if evaluation.get("Komplexität", 1.0) < 0.7:
            recommendations.append(
                "• Komplexität: Reduzieren Sie Abhängigkeiten zwischen Schritten"
            )

        if results["simulation_data"].get("collision_count", 0) > 5:
            recommendations.append(
                "• Kollisionen: Zu viele Kollisionen während der Montage"
            )

        if not recommendations:
            recommendations.append("• Keine spezifischen Verbesserungen erforderlich")

        return recommendations

    def shutdown(self) -> None:
        self.simulator.disconnect()
        logger.info("AssemblyFramework beendet")


# ---------------------------------------------------------------------------
# Beispiel-Daten
# ---------------------------------------------------------------------------


def create_example_components() -> List[Component]:
    components = [
        Component(
            id=1,
            name="Grundplatte",
            urdf_path="cube.urdf",
            mass=2.0,
            position=[0.0, 0.0, 0.1],
            dimensions=[0.4, 0.3, 0.05],
            connection_points=3,
            tools_required=["screwdriver"],
        ),
        Component(
            id=2,
            name="Träger_Links",
            urdf_path="cube.urdf",
            mass=0.5,
            position=[-0.15, 0.0, 0.2],
            dimensions=[0.05, 0.25, 0.15],
            connection_points=2,
            tools_required=["screwdriver", "wrench"],
        ),
        Component(
            id=3,
            name="Träger_Rechts",
            urdf_path="cube.urdf",
            mass=0.5,
            position=[0.15, 0.0, 0.2],
            dimensions=[0.05, 0.25, 0.15],
            connection_points=2,
            tools_required=["screwdriver", "wrench"],
        ),
        Component(
            id=4,
            name="Deckplatte",
            urdf_path="cube.urdf",
            mass=1.0,
            position=[0.0, 0.0, 0.35],
            dimensions=[0.3, 0.2, 0.03],
            connection_points=4,
            tools_required=["screwdriver", "allen_key"],
        ),
    ]
    return components


def create_example_sequence() -> AssemblySequence:
    steps = [
        AssemblyStep(
            step_id=1,
            component_id=1,
            target_position=[0.0, 0.0, 0.1],
            target_orientation=[0.0, 0.0, 0.0, 1.0],
            prerequisites=[],
            tools_used=["screwdriver"],
        ),
        AssemblyStep(
            step_id=2,
            component_id=2,
            target_position=[-0.15, 0.0, 0.2],
            target_orientation=[0.0, 0.0, 0.0, 1.0],
            prerequisites=[1],
            tools_used=["screwdriver", "wrench"],
        ),
        AssemblyStep(
            step_id=3,
            component_id=3,
            target_position=[0.15, 0.0, 0.2],
            target_orientation=[0.0, 0.0, 0.0, 1.0],
            prerequisites=[1],
            tools_used=["screwdriver", "wrench"],
        ),
        AssemblyStep(
            step_id=4,
            component_id=4,
            target_position=[0.0, 0.0, 0.35],
            target_orientation=[0.0, 0.0, 0.0, 1.0],
            prerequisites=[2, 3],
            tools_used=["screwdriver", "allen_key"],
        ),
    ]

    sequence = AssemblySequence(
        sequence_id=1,
        name="Standard_Montagereihenfolge",
        steps=steps,
        description="Beispielhafte Montagereihenfolge für Demonstrationszwecke",
    )
    return sequence


# ---------------------------------------------------------------------------
# Hauptfunktion
# ---------------------------------------------------------------------------


def main() -> None:
    print("Montagereihenfolge Validierungsframework")
    print("=" * 50)

    try:
        framework = AssemblyFramework(gui_mode=True)

        components = create_example_components()
        framework.components = components
        for component in components:
            framework.simulator.load_component(component)

        sequence = create_example_sequence()

        print(f"Bewerte Montagereihenfolge: {sequence.name}")
        results = framework.run_full_evaluation(sequence)

        report = framework.generate_report(sequence, results)
        print("\n" + report)

        framework.export_results_to_csv()

        print(f"\nErgebnisse gespeichert in: {framework.output_dir}")
        print("Simulation läuft... Drücken Sie Enter zum Beenden.")
        input()

    except Exception as e:
        logger.error("Fehler in der Hauptfunktion: %s", e)
        raise
    finally:
        if "framework" in locals():
            framework.shutdown()


if __name__ == "__main__":
    main()
