# -*- coding: utf-8 -*-
"""
Montagereihenfolge Validierungsframework für PyBullet
=====================================================


"""

import pybullet as p
import pybullet_data
import numpy as np
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

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("assembly_simulation.log"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)


@dataclass
class Component:
    id: int
    name: str
    urdf_path: str
    mass: float
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    dimensions: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    mesh_scale: List[float] = field(default_factory=lambda: [0.001, 0.001, 0.001])
    connection_points: int = 0
    tools_required: List[str] = field(default_factory=list)
    physics_id: Optional[int] = None


@dataclass
class AssemblyStep:
    step_id: int
    component_id: int
    target_position: List[float]
    target_orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    prerequisites: List[int] = field(default_factory=list)
    tools_used: List[str] = field(default_factory=list)


@dataclass
class AssemblySequence:
    sequence_id: int
    name: str
    steps: List[AssemblyStep]
    total_time: float = 0.0
    description: str = ""


class QualityCriterion(ABC):
    @abstractmethod
    def calculate(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> float:
        raise NotImplementedError

    @abstractmethod
    def get_name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def get_description(self) -> str:
        raise NotImplementedError


class TimeEfficiencyCriterion(QualityCriterion):
    def __init__(self, gamma: float = 0.5) -> None:
        self.gamma = gamma

    def calculate(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> float:
        n_parts = len(sequence.steps)
        if n_parts <= 1:
            return 1.0
        t_i = self._calculate_tree_depth(sequence)
        t_max = n_parts
        t_min = math.ceil(math.log2(n_parts)) if n_parts > 1 else 1
        d_para = 1.0 if t_max == t_min else (t_max - t_i) / (t_max - t_min)
        d_para = max(0.0, min(1.0, d_para))
        tool_conflict_ratio = self._compute_tool_conflicts(sequence, components)
        resource_factor = (1.0 - self.gamma) * (1.0 - tool_conflict_ratio)
        time_efficiency = d_para * self.gamma + resource_factor
        return max(0.0, min(1.0, time_efficiency))

    def _calculate_tree_depth(self, sequence: AssemblySequence) -> int:
        step_depths: Dict[int, int] = {}
        sorted_steps = sorted(sequence.steps, key=lambda s: s.step_id)
        for step in sorted_steps:
            if not step.prerequisites:
                step_depths[step.step_id] = 1
            else:
                prerequisite_depths = [step_depths.get(pr, 1) for pr in step.prerequisites]
                step_depths[step.step_id] = max(prerequisite_depths) + 1 if prerequisite_depths else 1
        return max(step_depths.values()) if step_depths else 1

    def _compute_tool_conflicts(self, sequence: AssemblySequence, components: List[Component]) -> float:
        comp_tools = {c.id: set(c.tools_required or []) for c in components}
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
                if step_j.step_id not in step_i.prerequisites and step_i.step_id not in step_j.prerequisites:
                    n_parallel_checks += 1
                    if step_tools[i] & step_tools[j]:
                        conflicts += 1
        if n_parallel_checks == 0:
            return 0.0
        return max(0.0, min(1.0, conflicts / n_parallel_checks))

    def get_name(self) -> str:
        return "Zeiteffizienz"

    def get_description(self) -> str:
        return "Bewertet die Zeiteffizienz basierend auf der Montageparallelität nach Zhao et al. (2012)."


class StabilityCriterion(QualityCriterion):
    def __init__(self) -> None:
        self.quantitative_weights = {
            "gravity_direction_count": 0.237,
            "positioning_bases": 0.186,
            "assembly_relationship": 0.2,
        }
        self.qualitative_weights = {
            "contact_type_quality": 0.13,
            "structural_stability": 0.248,
        }

    def calculate(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> float:
        quantitative = self._collect_quantitative_indicators(sequence, components, simulation_data)
        qualitative = self._collect_qualitative_indicators(sequence, components, simulation_data)
        norm_quant = self._normalize_indicators(quantitative)
        norm_qual = self._normalize_indicators(qualitative)
        weighted = self._apply_weights(norm_quant, norm_qual)
        stability_score = self._calculate_topsis_score(weighted)
        return max(0.0, min(1.0, stability_score if stability_score is not None else 0.5))

    def _collect_quantitative_indicators(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> Dict[str, float]:
        indicators: Dict[str, float] = {}
        gravity_aligned_count = 0
        for step in sequence.steps:
            target_pos = step.target_position
            if len(target_pos) >= 3:
                z_component = abs(target_pos[2])
                total_magnitude = math.sqrt(sum(v ** 2 for v in target_pos))
                if total_magnitude > 0 and z_component / total_magnitude > 0.7:
                    gravity_aligned_count += 1
        indicators["gravity_direction_count"] = gravity_aligned_count
        indicators["positioning_bases"] = sum(component.connection_points for component in components)
        indicators["assembly_relationship"] = simulation_data.get("established_contacts", len(sequence.steps))
        return indicators

    def _collect_qualitative_indicators(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> Dict[str, float]:
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
            contact_quality_scores.append((a + 2 * b + c_val) / 4.0)
        indicators["contact_type_quality"] = sum(contact_quality_scores) / len(contact_quality_scores) if contact_quality_scores else 0.5
        stability_factors: List[float] = []
        center_of_mass = simulation_data.get("center_of_mass", [0.0, 0.0, 0.5])
        com_height = center_of_mass[2] if len(center_of_mass) > 2 else 0.5
        com_score = max(0.0, 1.0 - min(1.0, com_height / 1.0))
        for base in [com_score,
                     max(0.0, 1.0 - min(1.0, simulation_data.get("collision_count", 0) / 10.0)),
                     max(0.0, 1.0 - min(1.0, simulation_data.get("max_joint_stress", 50.0) / 200.0))]:
            a = max(0.0, base - 0.1)
            c_val = min(1.0, base + 0.1)
            stability_factors.append((a + 2 * base + c_val) / 4.0)
        indicators["structural_stability"] = stability_factors[0] * 0.40 + stability_factors[1] * 0.35 + stability_factors[2] * 0.25
        return indicators

    def _normalize_indicators(self, indicators: Dict[str, float]) -> Dict[str, float]:
        normalized: Dict[str, float] = {}
        if not indicators:
            return normalized
        benefit_indicators = {"assembly_relationship", "gravity_direction_count", "positioning_bases"}
        for key, value in indicators.items():
            if value == 0:
                normalized[key] = 0.0
            elif key in benefit_indicators:
                max_value = max(indicators.values()) if indicators.values() else 1.0
                normalized[key] = value / max_value if max_value > 0 else 0.0
            else:
                min_value = min(v for v in indicators.values() if v > 0)
                normalized[key] = min_value / value if value > 0 else 0.0
        return normalized

    def _apply_weights(self, quantitative: Dict[str, float], qualitative: Dict[str, float]) -> Dict[str, float]:
        weighted: Dict[str, float] = {}
        for key, value in quantitative.items():
            weighted[key] = value * self.quantitative_weights.get(key, 0.0)
        for key, value in qualitative.items():
            weighted[key] = value * self.qualitative_weights.get(key, 0.0)
        return weighted

    def _calculate_topsis_score(self, weighted_indicators: Dict[str, float]) -> Optional[float]:
        if not weighted_indicators:
            return 0.5
        distance_positive = 0.0
        distance_negative = 0.0
        for value in weighted_indicators.values():
            distance_positive += (value - value) ** 2
            distance_negative += (value - 0.0) ** 2
        distance_positive = math.sqrt(distance_positive)
        distance_negative = math.sqrt(distance_negative)
        denominator = distance_positive + distance_negative
        if denominator == 0.0:
            return 1.0
        return distance_negative / denominator

    def get_name(self) -> str:
        return "Stabilität"

    def get_description(self) -> str:
        return "Bewertet die strukturelle Stabilität während der Montage nach Ma et al. (2015) mit einem TOPSIS-Ansatz."


class AccessibilityCriterion(QualityCriterion):
    def calculate(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> float:
        steps_data = simulation_data.get("steps", [])
        total_interference_events = 0
        free_direction_steps = 0
        clearance_scores: List[float] = []
        tool_scores: List[float] = []
        for step in sequence.steps:
            step_data = next((sd for sd in steps_data if sd.get("step_id") == step.step_id), {})
            collisions = step_data.get("collisions", [])
            k_step = len(collisions)
            total_interference_events += k_step
            if k_step == 0:
                free_direction_steps += 1
            clearance = simulation_data.get(f"clearance_step_{step.step_id}", None)
            if clearance is not None:
                clearance_mm = clearance * 1000.0
                c_min, c1, c2, c_max = 0.0, 10.0, 40.0, 80.0
                if clearance_mm <= c_min or clearance_mm >= c_max:
                    mu_c = 0.0
                elif c1 <= clearance_mm <= c2:
                    mu_c = 1.0
                elif c_min < clearance_mm < c1:
                    mu_c = (clearance_mm - c_min) / (c1 - c_min)
                else:
                    mu_c = (c_max - clearance_mm) / (c_max - c2)
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
        g_space = 1.0 if (l + k) == 0 else 1.0 - (k / (l + k))
        clearance_index = sum(clearance_scores) / len(clearance_scores) if clearance_scores else 1.0
        tool_index = sum(tool_scores) / len(tool_scores) if tool_scores else 1.0
        accessibility_score = (g_space + clearance_index + tool_index) / 3.0
        return max(0.0, min(1.0, accessibility_score))

    def get_name(self) -> str:
        return "Zugänglichkeit"

    def get_description(self) -> str:
        return "Bewertet die Zugänglichkeit auf Basis von Kollisionen, automatisch gemessenem Freiraum und automatisch geprüftem Werkzeugzugang."


class ComplexityCriterion(QualityCriterion):
    def calculate(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> float:
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
        p_pr = 0
        p_all = n * (n - 1) // 2
        if p_all == 0:
            return 1.0
        for i in range(n):
            for j in range(i + 1, n):
                pr_ij = 1 if precedes[i][j] and not precedes[j][i] else (-1 if precedes[j][i] and not precedes[i][j] else 0)
                if pr_ij != 0:
                    p_pr += 1
        os_value = p_pr / p_all
        return max(0.0, min(1.0, 1.0 - os_value))

    def get_name(self) -> str:
        return "Komplexität"

    def get_description(self) -> str:
        return "Bewertet die strukturelle Komplexität der Montagereihenfolge über die Assembly Sequence Flexibility (ASF)."


class QualityEvaluator:
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

    def evaluate_sequence(self, sequence: AssemblySequence, components: List[Component], simulation_data: Dict) -> Dict[str, float]:
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
        results["Gesamtbewertung"] = total_weighted_score / total_weight if total_weight > 0 else 0.0
        return results


class PyBulletSimulator:
    def __init__(self, gui_mode: bool = True, gravity: float = -9.81) -> None:
        self.gui_mode = gui_mode
        self.gravity = gravity
        self.physics_client: Optional[int] = None
        self.loaded_objects: Dict[int, int] = {}
        self.simulation_data: Dict[str, Any] = {}
        self.debug_item_ids: List[int] = []
        self._initialize_simulation()

    def _normalize_quaternion(self, q: List[float]) -> List[float]:
        norm = math.sqrt(sum(v * v for v in q))
        if norm <= 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [v / norm for v in q]

    def _parse_orientation(self, orientation: List[float]) -> List[float]:
        if not orientation or len(orientation) != 4:
            return [0.0, 0.0, 0.0, 1.0]
        x, y, z, w = orientation
        axis_len = math.sqrt(x * x + y * y + z * z)
        if abs(w) > 1.0:
            if axis_len <= 1e-12:
                return [0.0, 0.0, 0.0, 1.0]
            axis = [x / axis_len, y / axis_len, z / axis_len]
            angle_rad = math.radians(w)
            quat = p.getQuaternionFromAxisAngle(axis, angle_rad)
            return list(quat)
        return self._normalize_quaternion([x, y, z, w])

    def _initialize_simulation(self) -> None:
        if self.gui_mode:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        else:
            self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, self.gravity)
        p.setTimeStep(1.0 / 240.0)
        self.plane_id = p.loadURDF("plane.urdf")

    def load_component(self, component: Component) -> int:
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.abspath(os.path.join(base_dir, component.urdf_path))
            orientation = self._parse_orientation(component.orientation)
            if path.lower().endswith(".stl"):
                mesh_scale = component.mesh_scale or [0.001, 0.001, 0.001]
                collision_shape = p.createCollisionShape(p.GEOM_MESH, fileName=path, meshScale=mesh_scale)
                visual_shape = p.createVisualShape(p.GEOM_MESH, fileName=path, meshScale=mesh_scale, rgbaColor=[0.55, 0.55, 0.80, 1.0])
                object_id = p.createMultiBody(baseMass=component.mass, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=component.position, baseOrientation=orientation)
            else:
                object_id = p.loadURDF(path, basePosition=component.position, baseOrientation=orientation, useFixedBase=False)
            component.orientation = orientation
            component.physics_id = object_id
            self.loaded_objects[component.id] = object_id
            p.changeDynamics(object_id, -1, mass=component.mass, lateralFriction=1.5, contactStiffness=1000, contactDamping=100, spinningFriction=0.1, rollingFriction=0.2, restitution=0.01, linearDamping=2, angularDamping=3)
            return object_id
        except Exception as e:
            logger.error("Fehler beim Laden des Bauteils '%s': %s", component.name, e)
            return self._create_fallback_object(component)

    def _create_fallback_object(self, component: Component) -> int:
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d / 2.0 for d in component.dimensions])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[d / 2.0 for d in component.dimensions], rgbaColor=[0.5, 0.5, 0.8, 1.0])
        object_id = p.createMultiBody(baseMass=component.mass, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=component.position, baseOrientation=component.orientation)
        component.physics_id = object_id
        self.loaded_objects[component.id] = object_id
        return object_id

    def reset_simulation(self) -> None:
        p.resetSimulation()
        self.loaded_objects.clear()
        self.simulation_data.clear()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, self.gravity)
        p.setTimeStep(1.0 / 240.0)

    def disconnect(self) -> None:
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None

    def change_dynamics_all_objects(self) -> None:
        for objid in self.loaded_objects.values():
            p.changeDynamics(objid, -1, contactStiffness=1200.0, contactDamping=300.0, lateralFriction=3.0, spinningFriction=0.2, rollingFriction=0.3, restitution=0.0001, linearDamping=5, angularDamping=6)

    def fix_object_after_assembly(self, objectid: int, component_name: str) -> None:
        p.changeDynamics(objectid, -1, linearDamping=8.0, angularDamping=8.0, activationState=p.ACTIVATION_STATE_SLEEP)

    def _compute_min_clearance(self, objectid: int, search_distance: float = 0.2) -> float:
        distances: List[float] = []
        for other_id in self.loaded_objects.values():
            if other_id == objectid:
                continue
            closest = p.getClosestPoints(bodyA=objectid, bodyB=other_id, distance=search_distance)
            for contact in closest:
                distances.append(contact[8])
        plane_contacts = p.getClosestPoints(bodyA=objectid, bodyB=self.plane_id, distance=search_distance)
        for contact in plane_contacts:
            distances.append(contact[8])
        positive_distances = [d for d in distances if d >= 0.0]
        if positive_distances:
            return min(positive_distances)
        return search_distance

    def _check_tool_access_for_pose(self, objectid: int, pos: List[float], tool_length: float = 0.12, radial_offset: float = 0.02) -> bool:
        directions = [
            ([1, 0, 0], [0, radial_offset, 0]),
            ([-1, 0, 0], [0, radial_offset, 0]),
            ([0, 1, 0], [radial_offset, 0, 0]),
            ([0, -1, 0], [radial_offset, 0, 0]),
            ([0, 0, 1], [radial_offset, 0, 0]),
        ]
        blocked_count = 0
        for direction, offset in directions:
            start = [pos[0] - direction[0] * tool_length + offset[0], pos[1] - direction[1] * tool_length + offset[1], pos[2] - direction[2] * tool_length + offset[2]]
            end = [pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]]
            hit = p.rayTest(start, end)[0]
            hit_body = hit[0]
            hit_fraction = hit[2]
            if hit_body not in (-1, objectid) and hit_fraction < 0.999:
                blocked_count += 1
        return blocked_count < len(directions)

    def simulate_assembly_step(self, step: AssemblyStep, component: Component, duration: float = 2.0) -> Dict[str, Any]:
        if not hasattr(component, "physics_id") or component.physics_id is None:
            if component.id not in self.loaded_objects:
                logger.error("simulate_assembly_step: Bauteil %s (ID %d) nicht in loaded_objects", component.name, component.id)
                return {}
            objectid = self.loaded_objects[component.id]
        else:
            objectid = component.physics_id
        stepdata: Dict[str, Any] = {
            "step_id": step.step_id,
            "component_id": step.component_id,
            "start_time": time.time(),
            "collisions": [],
            "positions": [],
            "orientations": [],
            "min_clearance": float("inf"),
            "tool_access_samples": [],
            "tool_access_free": True,
        }
        startpos, startorn = p.getBasePositionAndOrientation(objectid)
        targetpos = step.target_position
        targetorn = self._parse_orientation(step.target_orientation)
        liftheight = 0.05
        waypoints = [
            [startpos[0], startpos[1], startpos[2] + liftheight],
            [targetpos[0], targetpos[1], targetpos[2] + liftheight],
            [targetpos[0], targetpos[1], targetpos[2]],
        ]
        segmentsteps = max(20, int(duration * 240.0 / len(waypoints)))
        currentstart = list(startpos)
        for waypoint_idx, waypoint in enumerate(waypoints):
            for i in range(segmentsteps):
                t = i / (segmentsteps - 1) if segmentsteps > 1 else 1.0
                currentpos = [currentstart[j] + t * (waypoint[j] - currentstart[j]) for j in range(3)]
                currentorn = targetorn if waypoint_idx == 2 else p.getQuaternionSlerp(startorn, targetorn, t)
                p.resetBasePositionAndOrientation(objectid, currentpos, currentorn)
                p.stepSimulation()
                if self.gui_mode:
                    time.sleep(1.0 / 480.0)
                pos, orn = p.getBasePositionAndOrientation(objectid)
                stepdata["positions"].append(pos)
                stepdata["orientations"].append(orn)
                min_clearance_now = self._compute_min_clearance(objectid, search_distance=0.2)
                stepdata["min_clearance"] = min(stepdata["min_clearance"], min_clearance_now)
                tool_access_now = self._check_tool_access_for_pose(objectid, list(pos))
                stepdata["tool_access_samples"].append(tool_access_now)
                contactpoints = p.getContactPoints(bodyA=objectid)
                if contactpoints:
                    stepdata["collisions"].append({"time": time.time(), "contactcount": len(contactpoints)})
            currentstart = waypoint
        for _ in range(80):
            p.stepSimulation()
            if self.gui_mode:
                time.sleep(1.0 / 480.0)
        self.fix_object_after_assembly(objectid, component.name)
        stepdata["tool_access_free"] = any(stepdata["tool_access_samples"]) if stepdata["tool_access_samples"] else True
        if math.isinf(stepdata["min_clearance"]):
            stepdata["min_clearance"] = 0.2
        stepdata["end_time"] = time.time()
        stepdata["duration"] = stepdata["end_time"] - stepdata["start_time"]
        stepdata[f"clearance_step_{step.step_id}"] = stepdata["min_clearance"]
        stepdata[f"tool_access_step_{step.step_id}"] = stepdata["tool_access_free"]
        return stepdata

    def get_simulation_data(self) -> Dict:
        data: Dict[str, Any] = {
            "timestamp": datetime.now().isoformat(),
            "loaded_objects": len(self.loaded_objects),
            "collision_count": 0,
            "center_of_mass": [0.0, 0.0, 0.0],
            "total_mass": 0.0,
        }
        total_mass = 0.0
        weighted_position = np.array([0.0, 0.0, 0.0])
        for objid in self.loaded_objects.values():
            contacts = p.getContactPoints(bodyA=objid)
            data["collision_count"] += len(contacts)
            dynamics_info = p.getDynamicsInfo(objid, -1)
            mass = float(dynamics_info[0])
            pos, _ = p.getBasePositionAndOrientation(objid)
            total_mass += mass
            weighted_position += np.array(pos) * mass
        if total_mass > 0.0:
            data["center_of_mass"] = (weighted_position / total_mass).tolist()
            data["total_mass"] = total_mass
        return data


class AssemblyFramework:
    def __init__(self, gui_mode: bool = True) -> None:
        self.simulator = PyBulletSimulator(gui_mode=gui_mode)
        self.evaluator = QualityEvaluator()
        self.components: List[Component] = []
        self.sequences: List[AssemblySequence] = []
        self.results: List[Dict[str, Any]] = []
        self.output_dir = "assembly_results"
        os.makedirs(self.output_dir, exist_ok=True)

    def load_components_from_file(self, file_path: str) -> None:
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.components.clear()
        for comp_data in data.get("components", []):
            component = Component(**comp_data)
            self.components.append(component)

    def load_sequence_from_file(self, file_path: str) -> None:
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

    def simulate_sequence(self, sequence: AssemblySequence) -> Dict:
        self.simulator.reset_simulation()
        for component in self.components:
            self.simulator.load_component(component)
        self.simulator.change_dynamics_all_objects()
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
                simulation_results["steps"].append({
                    "step_id": step.step_id,
                    "duration": 0.0,
                    "error": f"Component ID {step.component_id} not found",
                })
                continue
            step_result = self.simulator.simulate_assembly_step(step, component)
            simulation_results["steps"].append(step_result)
            simulation_results["total_duration"] += step_result.get("duration", 0.0)
            if "min_clearance" in step_result:
                simulation_results[f"clearance_step_{step.step_id}"] = step_result["min_clearance"]
            if "tool_access_free" in step_result:
                simulation_results[f"tool_access_step_{step.step_id}"] = step_result["tool_access_free"]
        final_data = self.simulator.get_simulation_data()
        simulation_results.update(final_data)
        simulation_results["end_time"] = datetime.now().isoformat()
        return simulation_results

    def evaluate_sequence(self, sequence: AssemblySequence, simulation_data: Dict) -> Dict[str, float]:
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

    def _save_results(self, results: Dict) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"assembly_results_{results['sequence_info']['name']}_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)

    def export_results_to_csv(self, filename: Optional[str] = None) -> None:
        if not self.results:
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
            fieldnames = list(csv_data[0].keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(csv_data)

    def shutdown(self) -> None:
        self.simulator.disconnect()


def create_example_components() -> List[Component]:
    return [
        Component(id=1, name="Grundplatte", urdf_path="cube.urdf", mass=2.0, position=[0.0, 0.0, 0.1], dimensions=[0.4, 0.3, 0.05], connection_points=3, tools_required=["screwdriver"]),
        Component(id=2, name="Träger_Links", urdf_path="cube.urdf", mass=0.5, position=[-0.15, 0.0, 0.2], dimensions=[0.05, 0.25, 0.15], connection_points=2, tools_required=["screwdriver", "wrench"]),
        Component(id=3, name="Träger_Rechts", urdf_path="cube.urdf", mass=0.5, position=[0.15, 0.0, 0.2], dimensions=[0.05, 0.25, 0.15], connection_points=2, tools_required=["screwdriver", "wrench"]),
        Component(id=4, name="Deckplatte", urdf_path="cube.urdf", mass=1.0, position=[0.0, 0.0, 0.35], dimensions=[0.3, 0.2, 0.03], connection_points=4, tools_required=["screwdriver", "allen_key"]),
    ]


def create_example_sequence() -> AssemblySequence:
    steps = [
        AssemblyStep(step_id=1, component_id=1, target_position=[0.0, 0.0, 0.1], target_orientation=[0.0, 0.0, 0.0, 1.0], prerequisites=[], tools_used=["screwdriver"]),
        AssemblyStep(step_id=2, component_id=2, target_position=[-0.15, 0.0, 0.2], target_orientation=[0.0, 0.0, 0.0, 1.0], prerequisites=[1], tools_used=["screwdriver", "wrench"]),
        AssemblyStep(step_id=3, component_id=3, target_position=[0.15, 0.0, 0.2], target_orientation=[0.0, 0.0, 0.0, 1.0], prerequisites=[1], tools_used=["screwdriver", "wrench"]),
        AssemblyStep(step_id=4, component_id=4, target_position=[0.0, 0.0, 0.35], target_orientation=[0.0, 0.0, 0.0, 1.0], prerequisites=[2, 3], tools_used=["screwdriver", "allen_key"]),
    ]
    return AssemblySequence(sequence_id=1, name="Standard_Montagereihenfolge", steps=steps, description="Beispielhafte Montagereihenfolge für Demonstrationszwecke")


def main() -> None:
    print("Montagereihenfolge Validierungsframework")
    print("=" * 50)
    framework = None
    try:
        framework = AssemblyFramework(gui_mode=True)
        framework.load_components_from_file("components.json")
        framework.load_sequence_from_file("sequence.json")
        sequence = framework.sequences[0]
        print(f"Bewerte Montagereihenfolge: {sequence.name}")
        results = framework.run_full_evaluation(sequence)
        print(json.dumps(results["evaluation_results"], ensure_ascii=False, indent=2))
        framework.export_results_to_csv()
        print(f"\nErgebnisse gespeichert in: {framework.output_dir}")
        print("Simulation läuft... Drücken Sie Enter zum Beenden.")
        input()
    except FileNotFoundError:
        print("components.json oder sequence.json nicht gefunden. Verwende Beispieldaten.")
        framework = AssemblyFramework(gui_mode=True)
        framework.components = create_example_components()
        sequence = create_example_sequence()
        results = framework.run_full_evaluation(sequence)
        print(json.dumps(results["evaluation_results"], ensure_ascii=False, indent=2))
        print("Simulation läuft... Drücken Sie Enter zum Beenden.")
        input()
    finally:
        if framework is not None:
            framework.shutdown()


if __name__ == "__main__":
    main()
