r"""An example to run of the minitaur gym environment with sine gaits.
"""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import math
import numpy as np
from pybullet_envs.bullet import minitaur_gym_env
import argparse
from pybullet_envs.bullet import minitaur_env_randomizer


# An example that the minitaur stands still using the reset pose.
def ResetPoseExample():
    """
    An example that the minitaur stands still using the reset pose.
    """
    # How many steps
    steps = 1000

    # Random number generator
    randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())

    # Minitaur gym environmennt
    environment = minitaur_gym_env.MinitaurBulletEnv(
        render=True,
        leg_model_enabled=False,
        motor_velocity_limit=np.inf,
        pd_control_enabled=True,
        accurate_motor_model_enabled=True,
        motor_overheat_protection=True,
        env_randomizer=randomizer,
        hard_reset=False
    )

    # Angle to apply at each step
    action = [math.pi / 2] * 8

    # Run simulation steps
    for _ in range(steps):
        _, _, done, _ = environment.step(action)
        if done:
            break
        # end if
    # end for

    # Reset the environment
    environment.reset()
# end ResetPoseExample


# An example of minitaur motor overheat protection is triggered.
def MotorOverheatExample():
    """
    An example of minitaur motor overheat protection is triggered.
    The minitaur is leaning forward and the motors are getting above threshold
    torques. The overheat protection will be triggered in ~1 sec.
    """
    environment = minitaur_gym_env.MinitaurBulletEnv(
        render=True,
        leg_model_enabled=False,
        motor_velocity_limit=np.inf,
        motor_overheat_protection=True,
        accurate_motor_model_enabled=True,
        motor_kp=1.20,
        motor_kd=0.00,
        on_rack=False
    )

    # Action to take at each step (empty)
    action = [.0] * 8

    # Fill actions
    for i in range(8):
        action[i] = .0 - 0.1 * (-1 if i % 2 == 0 else 1) * (-1 if i < 4 else 1)
    # end for

    # How many steps to simulate
    steps = 500

    # Stock action and observations
    actions_and_observations = []

    # For each step
    for step_counter in range(steps):
        # Matches the internal timestep
        time_step = 0.01

        # Temporal position
        t = step_counter * time_step

        # Time position and action
        current_row = [t]
        current_row.extend(action)

        # Result from action
        observation, _, _, _ = environment.step(action)

        # Add observation to row
        current_row.extend(observation.tolist())

        # Save actions and observations
        actions_and_observations.append(current_row)
    # end for

    # Reset environment
    environment.reset()
# end MotorOverHeatExample


# An example of minitaur standing and squatting on the floor.
def SineStandExample():
    """
    An example of minitaur standing and squatting on the floor.
    To validate the accurate motor model we command the robot and sit and stand up
    periodically in both simulation and experiment. We compare the measured motor
    trajectories, torques and gains.
    """
    # Load the environment
    environment = minitaur_gym_env.MinitaurBulletEnv(
        render=True,
        leg_model_enabled=False,
        motor_velocity_limit=np.inf,
        motor_overheat_protection=True,
        accurate_motor_model_enabled=True,
        motor_kp=1.20,
        motor_kd=0.02,
        on_rack=False
    )

    # How many steps, amplitude and steps
    steps = 1000
    amplitude = 0.5
    speed = 30

    # List to save actions and observations
    actions_and_observations = []

    # Run each steps
    for step_counter in range(steps):
        # Matches the internal time step
        time_step = 0.01

        # Time position
        t = step_counter * time_step

        # Add time
        current_row = [t]

        # Action as sine signal
        action = [math.sin(speed * t) * amplitude + math.pi / 2] * 8

        # Add action
        current_row.extend(action)

        # Make action an get the results
        observation, _, _, _ = environment.step(action)

        # Add the observation
        current_row.extend(observation.tolist())

        # Append to the list
        actions_and_observations.append(current_row)
    # end for

    environment.reset()
# end SineStandExample


# An example of minitaur walking with a sine gait
def SinePolicyExample():
    """
    An example of minitaur walking with a sine gait
    """
    # Random generator
    randomizer = (minitaur_env_randomizer.MinitaurEnvRandomizer())

    # Load the minitaur environment
    environment = minitaur_gym_env.MinitaurBulletEnv(
        render=True,
        motor_velocity_limit=np.inf,
        pd_control_enabled=True,
        hard_reset=False,
        env_randomizer=randomizer,
        on_rack=False
    )

    # Reward summation
    sum_reward = 0

    # How many steps
    steps = 20000

    # Amplitudes
    amplitude_1_bound = 0.1
    amplitude_2_bound = 0.1

    # Speed parametere
    speed = 10

    # Run each steps
    for step_counter in range(steps):
        # Time step
        time_step = 0.01

        # Temporal position
        t = step_counter * time_step

        # Amplitudes
        amplitude1 = amplitude_1_bound
        amplitude2 = amplitude_2_bound
        steering_amplitude = 0

        # Forward
        if t < 10:
            steering_amplitude = 0.1
        # Backward
        elif t < 20:
            steering_amplitude = -0.1
        # Nothing
        else:
            steering_amplitude = 0
        # end if

        # Applying asymmetrical sine gaits to different legs can steer the minitaur.
        a1 = math.sin(t * speed) * (amplitude1 + steering_amplitude)
        a2 = math.sin(t * speed + math.pi) * (amplitude1 - steering_amplitude)
        a3 = math.sin(t * speed) * amplitude2
        a4 = math.sin(t * speed + math.pi) * amplitude2

        # List of actions
        action = [a1, a2, a2, a1, a3, a4, a4, a3]

        # Apply action and get reward
        _, reward, done, _ = environment.step(action)

        # Add to total reward
        sum_reward += reward

        # If done, end
        if done:
            break
        # end if
    # end for

    # Reset the environment
    environment.reset()
# end SinePolicyExample


# Main function
def main():
    """
    Main function
    """
    # Parser with default help
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Add arguments to choose which policy to use
    parser.add_argument(
        '--env',
        help='environment ID (0==sine, 1==stand, 2=reset, 3=overheat)',
        type=int,
        default=0
    )

    # Parse argument
    args = parser.parse_args()

    # Launch selected example
    if args.env == 0:
        SinePolicyExample()
    if args.env == 1:
        SineStandExample()
    if args.env == 2:
        ResetPoseExample()
    if args.env == 3:
        MotorOverheatExample()
    # end if
# end main


if __name__ == '__main__':
  main()
# end if
