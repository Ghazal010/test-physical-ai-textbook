---
title: Week 10 - Sim-to-Real Transfer
description: Techniques for transferring simulation results to real robot deployment
sidebar_position: 2
---

# Week 10: Sim-to-Real Transfer

## Learning Objectives
- Understand the challenges and techniques in sim-to-real transfer
- Implement domain randomization to improve transferability
- Apply system identification methods to bridge simulation-reality gap
- Evaluate and validate sim-to-real transfer performance
- Design robust controllers that work in both simulation and reality

## Prerequisites Check
- Completion of Week 9 (Isaac ROS Integration and Perception Pipelines)
- Understanding of Isaac Sim and simulation environments
- Experience with robot control and navigation systems
- Knowledge of system identification concepts

## Theoretical Concepts: Sim-to-Real Transfer

### The Reality Gap Problem

Sim-to-real transfer, also known as domain transfer, addresses the fundamental challenge that behaviors learned or validated in simulation often fail when deployed on real robots. This "reality gap" arises from differences between simulated and real environments including:

- **Physical Properties**: Differences in friction, mass, inertia, and material properties
- **Sensor Characteristics**: Noise, latency, and accuracy variations between simulated and real sensors
- **Actuator Dynamics**: Response time, precision, and power limitations of real actuators
- **Environmental Conditions**: Lighting, temperature, air resistance, and other environmental factors
- **Modeling Limitations**: Simplifications and approximations in simulation models

### Domain Randomization

Domain randomization is a technique that increases the variability of simulation environments during training to make learned behaviors more robust to real-world variations. By training across diverse conditions, policies become less sensitive to specific simulation parameters.

Key aspects of domain randomization:
- **Visual Randomization**: Varying lighting, textures, colors, and camera properties
- **Physical Randomization**: Changing friction, mass, damping, and other physical parameters
- **Dynamics Randomization**: Adjusting actuator models, sensor noise, and control delays
- **Environmental Randomization**: Modifying floor textures, obstacles, and environmental conditions

### System Identification for Reality Gap Bridging

System identification involves modeling the real robot's dynamics based on observed behavior to update simulation parameters. This process helps align simulation with reality by identifying actual physical parameters.

Common approaches:
- **Parametric Identification**: Estimating specific physical parameters (mass, friction, etc.)
- **Non-parametric Identification**: Learning complete behavioral models from data
- **Online Adaptation**: Continuously updating models based on real-world performance

## Step-by-Step Tutorials: Sim-to-Real Transfer Techniques

### Domain Randomization Implementation

```python
import numpy as np
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import gym
from isaacgym import gymapi, gymtorch
import torch

@dataclass
class DomainRandomizationParams:
    """
    Parameters for domain randomization
    """
    # Physical properties randomization
    mass_range: Tuple[float, float] = (0.8, 1.2)  # Multiplier for mass
    friction_range: Tuple[float, float] = (0.5, 1.5)  # Multiplier for friction
    damping_range: Tuple[float, float] = (0.8, 1.2)  # Multiplier for damping

    # Visual properties randomization
    lighting_range: Tuple[float, float] = (0.5, 2.0)  # Light intensity multiplier
    texture_randomization: bool = True
    color_randomization: bool = True

    # Sensor noise parameters
    camera_noise_range: Tuple[float, float] = (0.0, 0.05)  # Noise level for cameras
    imu_noise_range: Tuple[float, float] = (0.0, 0.01)  # Noise level for IMU

    # Actuator dynamics randomization
    delay_range: Tuple[float, float] = (0.0, 0.05)  # Command delay in seconds
    accuracy_range: Tuple[float, float] = (0.95, 1.05)  # Command accuracy multiplier

class DomainRandomizationEnv:
    """
    Environment with domain randomization for sim-to-real transfer
    """

    def __init__(self, gym_instance, params: DomainRandomizationParams):
        self.gym = gym_instance
        self.params = params
        self.current_params = {}

        # Initialize randomization state
        self.reset_randomization()

    def reset_randomization(self):
        """
        Reset domain randomization parameters for new episode
        """
        self.current_params = {
            'mass_multiplier': np.random.uniform(*self.params.mass_range),
            'friction_multiplier': np.random.uniform(*self.params.friction_range),
            'damping_multiplier': np.random.uniform(*self.params.damping_range),
            'lighting_multiplier': np.random.uniform(*self.params.lighting_range),
            'camera_noise_level': np.random.uniform(*self.params.camera_noise_range),
            'imu_noise_level': np.random.uniform(*self.params.imu_noise_range),
            'command_delay': np.random.uniform(*self.params.delay_range),
            'actuator_accuracy': np.random.uniform(*self.params.accuracy_range)
        }

    def apply_randomization(self, env_ptr, actor_handle):
        """
        Apply domain randomization to a specific actor in the environment
        """
        # Randomize mass properties
        mass_props = self.gym.get_actor_rigid_body_properties(env_ptr, actor_handle)
        for mass_prop in mass_props:
            mass_prop.mass *= self.current_params['mass_multiplier']
            mass_prop.friction *= self.current_params['friction_multiplier']
            mass_prop.linear_damping *= self.current_params['damping_multiplier']
            mass_prop.angular_damping *= self.current_params['damping_multiplier']

        self.gym.set_actor_rigid_body_properties(env_ptr, actor_handle, mass_props)

        # Randomize visual properties (if applicable)
        if self.params.texture_randomization:
            self._randomize_textures(env_ptr, actor_handle)

        if self.params.color_randomization:
            self._randomize_colors(env_ptr, actor_handle)

    def _randomize_textures(self, env_ptr, actor_handle):
        """
        Randomize textures for the actor
        """
        # In Isaac Sim, this would involve changing material properties
        # This is a simplified example
        pass

    def _randomize_colors(self, env_ptr, actor_handle):
        """
        Randomize colors for the actor
        """
        # Change visual properties randomly
        pass

    def add_sensor_noise(self, sensor_data: Dict) -> Dict:
        """
        Add realistic noise to sensor data based on current randomization
        """
        noisy_data = sensor_data.copy()

        # Add camera noise
        if 'camera' in noisy_data:
            noise_level = self.current_params['camera_noise_level']
            noise = np.random.normal(0, noise_level, noisy_data['camera'].shape)
            noisy_data['camera'] = np.clip(noisy_data['camera'] + noise, 0, 1)

        # Add IMU noise
        if 'imu' in noisy_data:
            noise_level = self.current_params['imu_noise_level']
            for key in ['orientation', 'angular_velocity', 'linear_acceleration']:
                if key in noisy_data['imu']:
                    noise = np.random.normal(0, noise_level, noisy_data['imu'][key].shape)
                    noisy_data['imu'][key] += noise

        return noisy_data

    def apply_actuator_delay(self, commands: np.ndarray) -> np.ndarray:
        """
        Apply actuator delay based on current randomization
        """
        delay_steps = int(self.current_params['command_delay'] * 60)  # Assuming 60Hz control
        if delay_steps > 0:
            # In practice, you'd implement a proper delay buffer
            # This is a simplified example
            pass

        # Apply actuator accuracy
        accuracy = self.current_params['actuator_accuracy']
        noisy_commands = commands * np.random.uniform(accuracy[0], accuracy[1], commands.shape)

        return noisy_commands

# Example usage in a training loop
def train_with_domain_randomization():
    """
    Example training loop with domain randomization
    """
    params = DomainRandomizationParams()
    env = DomainRandomizationEnv(gymapi, params)

    for episode in range(10000):
        # Reset randomization for new episode
        env.reset_randomization()

        # Apply randomization to environment
        # ... setup environment with randomization ...

        # Train policy with randomized environment
        # ... training code ...

        if episode % 100 == 0:
            print(f"Episode {episode}, Randomization params: {env.current_params}")
```

### System Identification for Parameter Estimation

```python
import numpy as np
from scipy.optimize import minimize
from scipy import signal
import matplotlib.pyplot as plt
from typing import Callable, Dict, Any

class SystemIdentifier:
    """
    System identifier for bridging sim-to-real gap
    """

    def __init__(self):
        self.model_params = {}
        self.identification_data = []

    def collect_identification_data(self, real_robot, simulation, inputs: np.ndarray) -> Dict:
        """
        Collect data from both real robot and simulation for system identification
        """
        real_outputs = []
        sim_outputs = []

        for input_signal in inputs:
            # Apply input to real robot and record output
            real_output = real_robot.apply_input_and_measure(input_signal)
            real_outputs.append(real_output)

            # Apply same input to simulation and record output
            sim_output = simulation.apply_input_and_measure(input_signal)
            sim_outputs.append(sim_output)

            # Store data for identification
            self.identification_data.append({
                'input': input_signal,
                'real_output': real_output,
                'sim_output': sim_output
            })

        return {
            'inputs': inputs,
            'real_outputs': np.array(real_outputs),
            'sim_outputs': np.array(sim_outputs)
        }

    def identify_mass_properties(self, data: Dict) -> Dict:
        """
        Identify mass properties by comparing dynamic responses
        """
        inputs = data['inputs']
        real_outputs = data['real_outputs']
        sim_outputs = data['sim_outputs']

        # Define objective function to minimize difference between real and sim
        def objective(params):
            # params[0] = mass_multiplier, params[1] = friction_multiplier
            adjusted_sim_outputs = self._adjust_simulation(
                inputs, sim_outputs, mass_mult=params[0], friction_mult=params[1]
            )

            # Calculate error between real and adjusted simulation
            error = np.mean((real_outputs - adjusted_sim_outputs) ** 2)
            return error

        # Initial guess
        initial_guess = [1.0, 1.0]  # [mass_mult, friction_mult]

        # Optimize parameters
        result = minimize(objective, initial_guess, method='BFGS')

        identified_params = {
            'mass_multiplier': result.x[0],
            'friction_multiplier': result.x[1],
            'optimization_success': result.success,
            'final_error': result.fun
        }

        return identified_params

    def _adjust_simulation(self, inputs, sim_outputs, mass_mult=1.0, friction_mult=1.0):
        """
        Adjust simulation outputs based on identified parameters
        """
        # This is a simplified model - in practice, you'd have more complex adjustments
        adjusted_outputs = sim_outputs.copy()

        # Apply mass and friction corrections
        # This would involve re-running simulation with corrected parameters
        # For this example, we'll just scale the response
        adjusted_outputs *= mass_mult  # Simplified adjustment

        return adjusted_outputs

    def identify_dynamics_model(self, data: Dict) -> Dict:
        """
        Identify a dynamics model that bridges sim and real
        """
        inputs = data['inputs']
        real_outputs = data['real_outputs']
        sim_outputs = data['sim_outputs']

        # Calculate the difference between real and simulation
        residual = real_outputs - sim_outputs

        # Fit a model to the residual to capture the discrepancy
        # Using a simple linear model as example
        # In practice, you might use neural networks or other complex models
        A = np.column_stack([inputs, sim_outputs])  # Features
        b = residual.flatten()  # Target

        # Solve for correction parameters
        if len(A) > len(A[0]):  # Ensure we have more equations than unknowns
            correction_params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        else:
            # Not enough data, return identity correction
            correction_params = np.zeros(A.shape[1])

        return {
            'correction_params': correction_params,
            'residual_variance': np.var(residual),
            'model_rank': rank
        }

    def apply_correction(self, sim_output: np.ndarray, input_signal: np.ndarray,
                        correction_params: np.ndarray) -> np.ndarray:
        """
        Apply identified correction to simulation output
        """
        # Apply the correction model
        features = np.hstack([input_signal.flatten(), sim_output.flatten()])
        correction = np.dot(correction_params, features)

        corrected_output = sim_output + correction.reshape(sim_output.shape)

        return corrected_output

class RealityGapBridger:
    """
    Complete system for bridging reality gap using multiple techniques
    """

    def __init__(self):
        self.system_identifier = SystemIdentifier()
        self.domain_randomization_params = DomainRandomizationParams()
        self.correction_model = None

    def calibrate_simulation(self, real_robot, simulation):
        """
        Calibrate simulation using system identification
        """
        # Design identification inputs (e.g., step inputs, chirp signals)
        inputs = self._design_identification_inputs()

        # Collect data from both real and simulated systems
        data = self.system_identifier.collect_identification_data(real_robot, simulation, inputs)

        # Identify system parameters
        mass_params = self.system_identifier.identify_mass_properties(data)
        dynamics_params = self.system_identifier.identify_dynamics_model(data)

        # Update simulation parameters
        self._update_simulation_parameters(mass_params, dynamics_params)

        # Store correction model
        self.correction_model = dynamics_params

        return {
            'mass_params': mass_params,
            'dynamics_params': dynamics_params,
            'calibration_success': True
        }

    def _design_identification_inputs(self) -> np.ndarray:
        """
        Design informative inputs for system identification
        """
        # Create various input signals for identification
        inputs = []

        # Step inputs
        for amplitude in [0.1, 0.3, 0.5]:
            step_input = np.zeros(100)
            step_input[50:] = amplitude
            inputs.append(step_input)

        # Sine sweeps
        for freq in [0.5, 1.0, 2.0]:
            t = np.linspace(0, 10, 100)
            sine_sweep = 0.5 * np.sin(2 * np.pi * freq * t)
            inputs.append(sine_sweep)

        # Random signals
        for _ in range(3):
            random_signal = np.random.normal(0, 0.3, 100)
            inputs.append(random_signal)

        return np.array(inputs)

    def _update_simulation_parameters(self, mass_params: Dict, dynamics_params: Dict):
        """
        Update simulation with identified parameters
        """
        # Update domain randomization parameters based on identification
        self.domain_randomization_params.mass_range = (
            mass_params['mass_multiplier'] * 0.9,
            mass_params['mass_multiplier'] * 1.1
        )
        self.domain_randomization_params.friction_range = (
            mass_params['friction_multiplier'] * 0.9,
            mass_params['friction_multiplier'] * 1.1
        )

    def deploy_policy(self, policy, simulation, real_robot):
        """
        Deploy policy with sim-to-real transfer techniques
        """
        # First, calibrate the simulation
        calibration_result = self.calibrate_simulation(real_robot, simulation)

        # Test policy in calibrated simulation
        sim_performance = self._test_policy_in_simulation(policy, simulation)

        # If simulation performance is good, deploy to real robot with monitoring
        real_performance = self._deploy_with_monitoring(policy, real_robot)

        return {
            'calibration_result': calibration_result,
            'sim_performance': sim_performance,
            'real_performance': real_performance,
            'transfer_success': sim_performance['score'] * 0.8 <= real_performance['score']
        }

    def _test_policy_in_simulation(self, policy, simulation):
        """
        Test policy in simulation
        """
        # Run multiple episodes with domain randomization
        scores = []
        for _ in range(10):
            score = simulation.run_episode(policy)
            scores.append(score)

        return {
            'average_score': np.mean(scores),
            'std_score': np.std(scores),
            'scores': scores
        }

    def _deploy_with_monitoring(self, policy, real_robot):
        """
        Deploy policy to real robot with safety monitoring
        """
        # Implement safe deployment with monitoring
        score = real_robot.run_episode_with_monitoring(policy)

        return {
            'score': score,
            'safety_metrics': real_robot.get_safety_metrics()
        }
```

## Code Examples with Explanations

### Advanced Domain Randomization with Isaac Sim

```python
import omni
from pxr import Gf, UsdGeom, PhysxSchema
import carb
import numpy as np
from typing import Dict, List, Optional

class IsaacSimDomainRandomizer:
    """
    Advanced domain randomization implementation for Isaac Sim
    """

    def __init__(self, stage):
        self.stage = stage
        self.randomization_params = {}
        self.original_properties = {}

    def setup_domain_randomization(self, randomization_config: Dict):
        """
        Setup domain randomization based on configuration
        """
        self.randomization_config = randomization_config

        # Store original properties for reset
        self._store_original_properties()

        # Apply initial randomization
        self.randomize_domain()

    def _store_original_properties(self):
        """
        Store original properties of objects in the scene
        """
        # Iterate through all prims in the stage
        for prim in self.stage.Traverse():
            if prim.IsA(UsdGeom.Xform):
                # Store original transform
                xform_api = UsdGeom.Xformable(prim)
                self.original_properties[prim.GetPath().pathString] = {
                    'transform': xform_api.GetLocalTransformationMatrix(),
                    'properties': {}
                }

                # Store physics properties if applicable
                physx_api = PhysxSchema.PhysxRigidBodyAPI(prim)
                if physx_api.GetAppliedMass().GetAuthoredValue() is not None:
                    self.original_properties[prim.GetPath().pathString]['properties']['mass'] = \
                        physx_api.GetAppliedMass().Get()

    def randomize_domain(self):
        """
        Apply domain randomization to the scene
        """
        for prim_path, config in self.randomization_config.items():
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                continue

            # Randomize different properties based on config
            if 'mass' in config:
                self._randomize_mass(prim, config['mass'])

            if 'friction' in config:
                self._randomize_friction(prim, config['friction'])

            if 'color' in config:
                self._randomize_color(prim, config['color'])

            if 'position' in config:
                self._randomize_position(prim, config['position'])

    def _randomize_mass(self, prim, mass_config: Dict):
        """
        Randomize mass of a rigid body
        """
        physx_api = PhysxSchema.PhysxRigidBodyAPI(prim)
        if not physx_api:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx_api = PhysxSchema.PhysxRigidBodyAPI(prim)

        original_mass = physx_api.GetAppliedMass().Get() or 1.0
        random_multiplier = np.random.uniform(
            mass_config['min_multiplier'],
            mass_config['max_multiplier']
        )
        new_mass = original_mass * random_multiplier

        physx_api.GetAppliedMassAttr().Set(new_mass)

    def _randomize_friction(self, prim, friction_config: Dict):
        """
        Randomize friction properties
        """
        # Apply PhysxMaterialAPI if not already applied
        material_api = PhysxSchema.PhysxMaterialAPI(prim)
        if not material_api:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)
            material_api = PhysxSchema.PhysxMaterialAPI(prim)

        static_friction = np.random.uniform(
            friction_config['min_static'],
            friction_config['max_static']
        )
        dynamic_friction = np.random.uniform(
            friction_config['min_dynamic'],
            friction_config['max_dynamic']
        )

        material_api.GetStaticFrictionAttr().Set(static_friction)
        material_api.GetDynamicFrictionAttr().Set(dynamic_friction)

    def _randomize_color(self, prim, color_config: Dict):
        """
        Randomize visual color properties
        """
        # This would involve modifying material properties in USD
        # Implementation depends on the material system used
        pass

    def _randomize_position(self, prim, position_config: Dict):
        """
        Randomize position of an object
        """
        xform_api = UsdGeom.Xformable(prim)
        original_transform = xform_api.GetLocalTransformationMatrix()

        # Add random offset
        random_offset = Gf.Vec3f(
            np.random.uniform(position_config['min_x'], position_config['max_x']),
            np.random.uniform(position_config['min_y'], position_config['max_y']),
            np.random.uniform(position_config['min_z'], position_config['max_z'])
        )

        # Apply new transform
        new_transform = original_transform
        new_transform.SetTranslateOnly(new_transform.GetTranslate() + random_offset)
        xform_api.SetLocalTransformation(new_transform)

    def reset_to_original(self):
        """
        Reset all randomized properties to original values
        """
        for prim_path, original_props in self.original_properties.items():
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                # Reset transform
                xform_api = UsdGeom.Xformable(prim)
                xform_api.SetLocalTransformationMatrix(original_props['transform'])

                # Reset properties
                if 'properties' in original_props:
                    for prop_name, value in original_props['properties'].items():
                        if prop_name == 'mass':
                            physx_api = PhysxSchema.PhysxRigidBodyAPI(prim)
                            physx_api.GetAppliedMassAttr().Set(value)
```

### Reality Gap Evaluation and Monitoring

```python
import time
import numpy as np
from typing import Dict, List, Tuple
from dataclasses import dataclass

@dataclass
class PerformanceMetrics:
    """
    Data class for storing performance metrics
    """
    sim_score: float
    real_score: float
    transfer_gap: float
    success_rate: float
    safety_metrics: Dict[str, float]
    time_metrics: Dict[str, float]

class RealityGapEvaluator:
    """
    System for evaluating and monitoring reality gap
    """

    def __init__(self):
        self.metrics_history = []
        self.warning_threshold = 0.2  # 20% performance drop threshold
        self.safety_limits = {
            'max_velocity': 2.0,  # m/s
            'max_acceleration': 5.0,  # m/s²
            'max_torque': 100.0,  # Nm
        }

    def evaluate_transfer(self, policy, sim_env, real_env, num_episodes: int = 10) -> PerformanceMetrics:
        """
        Evaluate sim-to-real transfer performance
        """
        sim_scores = []
        real_scores = []
        safety_violations = []
        execution_times = []

        for episode in range(num_episodes):
            # Test in simulation
            start_time = time.time()
            sim_score = self._run_episode(policy, sim_env)
            sim_scores.append(sim_score)
            sim_time = time.time() - start_time

            # Test in reality
            start_time = time.time()
            real_score, safety_metrics = self._run_episode_with_safety(policy, real_env)
            real_scores.append(real_score)
            real_time = time.time() - start_time
            execution_times.append({'sim': sim_time, 'real': real_time})
            safety_violations.append(safety_metrics)

        # Calculate metrics
        avg_sim_score = np.mean(sim_scores)
        avg_real_score = np.mean(real_scores)
        transfer_gap = (avg_sim_score - avg_real_score) / avg_sim_score if avg_sim_score != 0 else 0
        success_rate = np.sum(np.array(real_scores) > 0) / len(real_scores)  # Assuming positive scores indicate success

        # Aggregate safety metrics
        aggregated_safety = {}
        for key in safety_violations[0].keys():
            aggregated_safety[key] = np.mean([v[key] for v in safety_violations])

        metrics = PerformanceMetrics(
            sim_score=avg_sim_score,
            real_score=avg_real_score,
            transfer_gap=transfer_gap,
            success_rate=success_rate,
            safety_metrics=aggregated_safety,
            time_metrics={
                'avg_sim_time': np.mean([t['sim'] for t in execution_times]),
                'avg_real_time': np.mean([t['real'] for t in execution_times])
            }
        )

        self.metrics_history.append(metrics)

        return metrics

    def _run_episode(self, policy, env):
        """
        Run a single episode with the given policy
        """
        obs = env.reset()
        total_reward = 0
        done = False

        while not done:
            action = policy.get_action(obs)
            obs, reward, done, info = env.step(action)
            total_reward += reward

        return total_reward

    def _run_episode_with_safety(self, policy, env):
        """
        Run episode with safety monitoring
        """
        obs = env.reset()
        total_reward = 0
        done = False
        safety_metrics = {
            'max_velocity_violations': 0,
            'max_acceleration_violations': 0,
            'max_torque_violations': 0,
            'safety_interventions': 0
        }

        while not done:
            action = policy.get_action(obs)

            # Check safety before applying action
            safety_ok, violations = self._check_safety(env, action)

            if not safety_ok:
                # Apply safety intervention
                safe_action = self._get_safe_action(env, action, violations)
                safety_metrics['safety_interventions'] += 1
            else:
                safe_action = action

            obs, reward, done, info = env.step(safe_action)
            total_reward += reward

            # Update safety metrics
            for violation_type in violations:
                if violation_type in safety_metrics:
                    safety_metrics[violation_type] += 1

        return total_reward, safety_metrics

    def _check_safety(self, env, action) -> Tuple[bool, List[str]]:
        """
        Check if action is safe to execute
        """
        violations = []

        # Predict next state based on action
        predicted_state = env.predict_next_state(action)

        # Check various safety constraints
        if predicted_state.velocity > self.safety_limits['max_velocity']:
            violations.append('max_velocity_violations')

        if predicted_state.acceleration > self.safety_limits['max_acceleration']:
            violations.append('max_acceleration_violations')

        if predicted_state.torque > self.safety_limits['max_torque']:
            violations.append('max_torque_violations')

        return len(violations) == 0, violations

    def _get_safe_action(self, env, original_action, violations):
        """
        Get a safe action that respects safety limits
        """
        # For now, return a conservative action
        # In practice, this would involve optimization
        safe_action = original_action * 0.5  # Reduce action magnitude
        return safe_action

    def generate_transfer_report(self) -> Dict:
        """
        Generate a comprehensive transfer report
        """
        if not self.metrics_history:
            return {"error": "No metrics history available"}

        latest_metrics = self.metrics_history[-1]

        report = {
            "transfer_analysis": {
                "latest_sim_score": latest_metrics.sim_score,
                "latest_real_score": latest_metrics.real_score,
                "transfer_gap_percentage": latest_metrics.transfer_gap * 100,
                "success_rate": latest_metrics.success_rate
            },
            "safety_assessment": {
                "max_velocity_violations": latest_metrics.safety_metrics.get('max_velocity_violations', 0),
                "max_acceleration_violations": latest_metrics.safety_metrics.get('max_acceleration_violations', 0),
                "safety_interventions": latest_metrics.safety_metrics.get('safety_interventions', 0)
            },
            "recommendations": []
        }

        # Add recommendations based on metrics
        if latest_metrics.transfer_gap > self.warning_threshold:
            report["recommendations"].append(
                f"High transfer gap detected ({latest_metrics.transfer_gap:.2%}). "
                "Consider improving domain randomization or system identification."
            )

        if latest_metrics.success_rate < 0.8:
            report["recommendations"].append(
                f"Low success rate ({latest_metrics.success_rate:.2%}). "
                "Policy may need additional training or safety improvements."
            )

        if latest_metrics.safety_metrics.get('safety_interventions', 0) > 5:
            report["recommendations"].append(
                "High number of safety interventions detected. "
                "Review safety limits and policy robustness."
            )

        return report
```

## Hands-On Exercises: Sim-to-Real Transfer Implementation

### Exercise 1: Domain Randomization for Object Manipulation

Implement domain randomization for a simple object manipulation task:

```python
import numpy as np
import random
from typing import Dict, Tuple
import gym
from gym import spaces

class RandomizedManipulationEnv(gym.Env):
    """
    Exercise: Object manipulation environment with domain randomization
    """

    def __init__(self):
        super(RandomizedManipulationEnv, self).__init__()

        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)  # dx, dy, dz, grip
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)

        # Randomization parameters
        self.randomization_params = {
            'object_mass_range': (0.05, 0.15),  # kg
            'friction_range': (0.1, 0.8),
            'object_size_range': (0.02, 0.05),  # meters
            'gripper_force_range': (5.0, 20.0)  # Newtons
        }

        # Current randomized parameters
        self.current_params = {}

        # Reset environment to apply initial randomization
        self.reset()

    def reset(self):
        """
        Reset environment with new randomization
        """
        # Apply new randomization
        self._apply_randomization()

        # Initialize object position randomly
        self.object_pos = np.array([
            np.random.uniform(-0.3, 0.3),
            np.random.uniform(-0.3, 0.3),
            np.random.uniform(0.1, 0.2)
        ])

        # Initialize gripper position
        self.gripper_pos = np.array([0.0, 0.0, 0.3])  # Start above object

        # Initialize object state
        self.object_mass = self.current_params['mass']
        self.object_friction = self.current_params['friction']
        self.object_size = self.current_params['size']
        self.gripper_force = self.current_params['gripper_force']

        # Initialize episode state
        self.episode_step = 0
        self.object_grasped = False

        return self._get_observation()

    def _apply_randomization(self):
        """
        Apply domain randomization to environment parameters
        """
        self.current_params = {
            'mass': np.random.uniform(*self.randomization_params['object_mass_range']),
            'friction': np.random.uniform(*self.randomization_params['friction_range']),
            'size': np.random.uniform(*self.randomization_params['object_size_range']),
            'gripper_force': np.random.uniform(*self.randomization_params['gripper_force_range'])
        }

    def step(self, action):
        """
        Execute one step of the environment
        """
        # Apply action to gripper (simplified physics)
        self.gripper_pos += action[:3] * 0.01  # Scale down the action

        # Check if gripper is close enough to object to grasp
        dist_to_object = np.linalg.norm(self.gripper_pos - self.object_pos)

        # Attempt grasp if close enough and gripper command is positive
        if dist_to_object < 0.05 and action[3] > 0.5 and not self.object_grasped:
            self.object_grasped = True

        # If object is grasped, move it with gripper
        if self.object_grasped:
            self.object_pos = self.gripper_pos.copy()

        # Calculate reward
        reward = self._calculate_reward()

        # Check if episode is done
        done = self._check_done()

        # Increment step counter
        self.episode_step += 1

        return self._get_observation(), reward, done, {}

    def _calculate_reward(self):
        """
        Calculate reward for current state
        """
        reward = 0.0

        # Reward for getting close to object
        dist_to_object = np.linalg.norm(self.gripper_pos - self.object_pos)
        reward += max(0, 1.0 - dist_to_object) * 0.5

        # Reward for grasping object
        if self.object_grasped:
            reward += 2.0

        # Reward for moving object to target (fixed target at [0.2, 0.2, 0.2])
        target_pos = np.array([0.2, 0.2, 0.2])
        if self.object_grasped:
            dist_to_target = np.linalg.norm(self.object_pos - target_pos)
            reward += max(0, 2.0 - dist_to_target)

        # Small penalty for taking too many steps
        reward -= 0.01

        return reward

    def _check_done(self):
        """
        Check if episode is done
        """
        # Episode ends after 100 steps or if object is successfully placed
        target_pos = np.array([0.2, 0.2, 0.2])
        if self.object_grasped:
            dist_to_target = np.linalg.norm(self.object_pos - target_pos)
            if dist_to_target < 0.05:  # Successfully placed
                return True

        return self.episode_step >= 100

    def _get_observation(self):
        """
        Get current observation
        """
        # Concatenate relevant state information
        obs = np.concatenate([
            self.gripper_pos,           # 3: gripper position
            self.object_pos,            # 3: object position
            [self.object_grasped],      # 1: grasp status
            [self.object_mass],         # 1: object mass (randomized)
            [self.object_friction],     # 1: object friction (randomized)
            [self.gripper_force]        # 1: gripper force (randomized)
        ])
        return obs

    def get_current_params(self):
        """
        Get current randomization parameters
        """
        return self.current_params

# Example training loop with randomization
def train_with_randomization():
    """
    Example training loop demonstrating domain randomization
    """
    env = RandomizedManipulationEnv()

    print("Training with domain randomization...")

    for episode in range(1000):
        obs = env.reset()  # New randomization applied at each reset
        episode_reward = 0
        step_count = 0

        while True:
            # Simple random policy for demonstration
            action = np.random.uniform(-1, 1, size=env.action_space.shape)

            obs, reward, done, info = env.step(action)
            episode_reward += reward
            step_count += 1

            if done:
                break

        # Print randomization parameters for this episode
        params = env.get_current_params()

        if episode % 100 == 0:
            print(f"Episode {episode}: Reward={episode_reward:.2f}, "
                  f"Mass={params['mass']:.3f}, Friction={params['friction']:.3f}, "
                  f"Size={params['size']:.3f}")

    print("Training completed!")
```

### Exercise 2: System Identification for Dynamics Calibration

Create a system identification system to calibrate simulation parameters:

```python
import numpy as np
from scipy.optimize import minimize
from scipy import signal
import matplotlib.pyplot as plt
from typing import Tuple, Dict

class DynamicsCalibrator:
    """
    Exercise: System identification for dynamics calibration
    """

    def __init__(self):
        self.nominal_params = {
            'mass': 1.0,      # kg
            'damping': 0.1,   # Ns/m
            'stiffness': 10.0  # N/m
        }
        self.identified_params = self.nominal_params.copy()

    def simulate_system(self, params: Dict, input_signal: np.ndarray,
                       dt: float = 0.01) -> np.ndarray:
        """
        Simulate a second-order system with given parameters
        """
        mass = params['mass']
        damping = params['damping']
        stiffness = params['stiffness']

        # Initialize state
        position = 0.0
        velocity = 0.0
        positions = []

        for u in input_signal:
            # Simple forward Euler integration
            acceleration = (u - damping * velocity - stiffness * position) / mass
            velocity += acceleration * dt
            position += velocity * dt

            positions.append(position)

        return np.array(positions)

    def collect_system_data(self, real_system_func, input_signal: np.ndarray) -> Dict:
        """
        Collect input-output data from real system
        """
        # Apply input to real system and measure output
        real_output = real_system_func(input_signal)

        # Apply same input to nominal simulation
        nominal_output = self.simulate_system(self.nominal_params, input_signal)

        return {
            'input': input_signal,
            'real_output': real_output,
            'nominal_output': nominal_output
        }

    def identify_parameters(self, data: Dict) -> Dict:
        """
        Identify system parameters by minimizing simulation-error
        """
        input_signal = data['input']
        real_output = data['real_output']

        def objective(params_array):
            # Convert array to parameter dictionary
            identified_params = {
                'mass': params_array[0],
                'damping': params_array[1],
                'stiffness': params_array[2]
            }

            # Simulate with identified parameters
            sim_output = self.simulate_system(identified_params, input_signal)

            # Calculate error
            error = np.mean((real_output - sim_output) ** 2)
            return error

        # Initial guess (start with nominal parameters)
        initial_guess = [
            self.nominal_params['mass'],
            self.nominal_params['damping'],
            self.nominal_params['stiffness']
        ]

        # Bounds for parameters (mass > 0, damping > 0, stiffness > 0)
        bounds = [(0.1, 5.0), (0.01, 1.0), (1.0, 50.0)]

        # Optimize parameters
        result = minimize(objective, initial_guess, method='L-BFGS-B', bounds=bounds)

        # Store identified parameters
        self.identified_params = {
            'mass': result.x[0],
            'damping': result.x[1],
            'stiffness': result.x[2]
        }

        return {
            'identified_params': self.identified_params,
            'optimization_success': result.success,
            'final_error': result.fun,
            'nfev': result.nfev
        }

    def validate_calibration(self, data: Dict) -> Dict:
        """
        Validate the calibrated parameters
        """
        input_signal = data['input']
        real_output = data['real_output']

        # Simulate with calibrated parameters
        calibrated_output = self.simulate_system(self.identified_params, input_signal)

        # Calculate validation metrics
        mse = np.mean((real_output - calibrated_output) ** 2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(real_output - calibrated_output))

        # Calculate fit percentage
        ss_res = np.sum((real_output - calibrated_output) ** 2)
        ss_tot = np.sum((real_output - np.mean(real_output)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0

        return {
            'mse': mse,
            'rmse': rmse,
            'mae': mae,
            'r_squared': r_squared,
            'calibrated_output': calibrated_output
        }

    def design_identification_signal(self, duration: float = 10.0,
                                   dt: float = 0.01) -> np.ndarray:
        """
        Design an informative input signal for system identification
        """
        t = np.arange(0, duration, dt)

        # Combine multiple signal types for rich excitation
        # 1. Pseudo-random binary sequence (PRBS) like signal
        prbs = np.sign(np.random.random(len(t)) - 0.5)

        # 2. Multi-sine signal with different frequencies
        multi_sine = np.zeros_like(t)
        for freq in [0.5, 1.0, 2.0, 5.0]:
            multi_sine += 0.3 * np.sin(2 * np.pi * freq * t)

        # 3. Step inputs at different times
        steps = np.zeros_like(t)
        for step_time in [2.0, 6.0]:
            steps[t >= step_time] += 0.5

        # Combine signals and normalize
        combined_signal = prbs * 0.3 + multi_sine * 0.5 + steps * 0.2
        combined_signal = np.clip(combined_signal, -1.0, 1.0)

        return combined_signal

def simulate_real_system(input_signal: np.ndarray, dt: float = 0.01) -> np.ndarray:
    """
    Simulate a "real" system with unknown parameters (for demonstration)
    """
    # True parameters of the "real" system
    true_params = {'mass': 1.2, 'damping': 0.15, 'stiffness': 8.5}

    # Add some noise to make it more realistic
    noise_level = 0.01

    calibrator = DynamicsCalibrator()
    output = calibrator.simulate_system(true_params, input_signal, dt)

    # Add measurement noise
    noisy_output = output + np.random.normal(0, noise_level, output.shape)

    return noisy_output

def main_calibration_example():
    """
    Main example demonstrating system identification
    """
    calibrator = DynamicsCalibrator()

    # Design identification signal
    input_signal = calibrator.design_identification_signal(duration=10.0)

    # Collect data from "real" system
    data = calibrator.collect_system_data(simulate_real_system, input_signal)

    print("Original nominal parameters:", calibrator.nominal_params)

    # Identify parameters
    identification_result = calibrator.identify_parameters(data)
    print("Identified parameters:", calibrator.identified_params)
    print("Optimization success:", identification_result['optimization_success'])
    print("Final error:", identification_result['final_error'])

    # Validate calibration
    validation_result = calibrator.validate_calibration(data)
    print("Validation metrics:")
    print(f"  RMSE: {validation_result['rmse']:.4f}")
    print(f"  MAE: {validation_result['mae']:.4f}")
    print(f"  R²: {validation_result['r_squared']:.4f}")

    # Plot results
    t = np.arange(len(data['input'])) * 0.01

    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(t, data['real_output'], label='Real System Output', linewidth=2)
    plt.plot(t, data['nominal_output'], label='Nominal Simulation', linestyle='--')
    plt.plot(t, validation_result['calibrated_output'], label='Calibrated Simulation', linestyle='-.')
    plt.title('System Identification Results')
    plt.xlabel('Time [s]')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(t, data['real_output'] - validation_result['calibrated_output'],
             label='Error (Real - Calibrated)', color='red')
    plt.title('Calibration Error')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main_calibration_example()
```

## Common Pitfalls and Solutions

### Pitfall 1: Insufficient Domain Randomization Coverage
**Problem**: Randomization ranges are too narrow, not covering real-world variations.

**Solution**:
- Use wide randomization ranges based on real-world measurements
- Continuously update randomization based on real robot performance
- Implement adaptive domain randomization

```python
class AdaptiveDomainRandomizer:
    """
    Adaptive domain randomizer that adjusts ranges based on real performance
    """

    def __init__(self, initial_ranges: Dict):
        self.ranges = initial_ranges
        self.performance_history = []
        self.adaptation_threshold = 0.1  # Performance drop threshold

    def update_randomization_ranges(self, real_performance: float, sim_performance: float):
        """
        Update randomization ranges based on performance gap
        """
        performance_gap = (sim_performance - real_performance) / sim_performance

        if performance_gap > self.adaptation_threshold:
            # Increase randomization ranges to cover more variation
            for param_name, (min_val, max_val) in self.ranges.items():
                center = (min_val + max_val) / 2
                range_width = max_val - min_val

                # Increase range by 10%
                new_width = range_width * 1.1
                self.ranges[param_name] = (
                    center - new_width / 2,
                    center + new_width / 2
                )

            print(f"Adapted randomization ranges due to performance gap: {performance_gap:.3f}")
```

### Pitfall 2: Overfitting to Simulation
**Problem**: Policies become too specialized to specific simulation conditions.

**Solution**:
- Use progressive domain randomization (start narrow, expand over time)
- Implement reality checking during training
- Regularly validate on real robot when possible

### Pitfall 3: Computational Overhead of Randomization
**Problem**: Excessive randomization slows down simulation training.

**Solution**:
- Randomize only critical parameters
- Use efficient randomization techniques
- Apply randomization only at episode resets, not every step

## Review Questions

1. What is the "reality gap" and why does it occur in robotics?
2. How does domain randomization help with sim-to-real transfer?
3. What is system identification and how is it used in sim-to-real transfer?
4. What are the main challenges in implementing effective sim-to-real transfer?
5. How can you evaluate the success of sim-to-real transfer?

## Project Assignment: Complete Sim-to-Real System

Create a complete sim-to-real transfer system that:
1. Implements domain randomization for a robotic task (navigation or manipulation)
2. Performs system identification to calibrate simulation parameters
3. Evaluates transfer performance with safety monitoring
4. Generates comprehensive reports on transfer quality
5. Implements adaptive techniques based on real-world performance

Your system should include:
- Configurable domain randomization for multiple parameters
- Automated system identification pipeline
- Safety monitoring during real robot deployment
- Performance evaluation and reporting
- Adaptive mechanisms to improve transfer over time

## Further Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [Domain Randomization in Robotics](https://arxiv.org/abs/1703.06907)
- [System Identification for Robotics](https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
- [Sim-to-Real Transfer Techniques](https://arxiv.org/list/cs.RO/recent)
- [Isaac ROS Bridge Documentation](https://nvidia-isaac-ros.github.io/concepts/isaac_sim_bridge/index.html)

:::warning
Always prioritize safety when transferring policies from simulation to real robots. Implement robust safety checks and start with conservative parameters to prevent damage to the robot or environment.
:::