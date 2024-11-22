#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementations.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import signal
import sys
import time

import py_trees
import carla
import threading

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.autoagents.agent_wrapper import AgentWrapperFactory, AgentError, TickRuntimeError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.result_writer import ResultOutputProvider


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, run and stop a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, timeout, agent_timeout,simulation_time_thd,frame_rate, statistics_manager, debug_mode=0,
                 use_predefined_route=False, other_agent_setting=False):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        self.route_index = None
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent_wrapper = None
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)
        self._agent_timeout = float(agent_timeout)
        self._simulation_time_thd = int(simulation_time_thd)
        self._frame_rate = float(frame_rate)
        self._tick_count_thd = self._frame_rate * simulation_time_thd  #The maximum simulation time is 12s.

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = 0.0
        self.start_game_time = 0.0
        self.end_system_time = 0.0
        self.end_game_time = 0.0

        self._watchdog = None
        self._agent_watchdog = None
        self._scenario_thread = None

        self._statistics_manager = statistics_manager

        self.tick_count = 0

        self.use_predefined_route = use_predefined_route
        self.other_agent_setting = other_agent_setting

        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Agent took longer than {}s to send its command".format(self._agent_timeout))
        elif self._watchdog and not self._watchdog.get_status():
            raise RuntimeError("The simulation took longer than {}s to update".format(self._timeout))
        self._running = False

    def cleanup(self):
        """
        Reset all parameters
        """
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = 0.0
        self.start_game_time = 0.0
        self.end_system_time = 0.0
        self.end_game_time = 0.0

        self._spectator = None
        self._watchdog = None
        self._agent_watchdog = None

    def load_scenario(self, scenario, agent, route_index, rep_number):
        """
        Load a new scenario
        """

        GameTime.restart()
        self._agent_wrapper = AgentWrapperFactory.get_wrapper(agent)
        self.route_index = route_index
        self.scenario = scenario
        self.scenario_tree = scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.repetition_number = rep_number

        self._spectator = CarlaDataProvider.get_world().get_spectator()

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        self._agent_wrapper.setup_sensors(self.ego_vehicles[0],self.use_predefined_route)

    def build_scenarios_loop(self, debug):
        """
        Keep periodically trying to start the scenarios that are close to the ego vehicle
        Additionally, do the same for the spawned vehicles
        """
        while self._running:
            self.scenario.build_scenarios(self.ego_vehicles[0], debug=debug)
            self.scenario.spawn_parked_vehicles(self.ego_vehicles[0])
            time.sleep(1)

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()

        # Detects if the simulation is down
        self._watchdog = Watchdog(self._timeout)
        self._watchdog.start()

        # Stop the agent from freezing the simulation
        self._agent_watchdog = Watchdog(self._agent_timeout)
        self._agent_watchdog.start()

        self._running = True

        # # Thread for build_scenarios
        # self._scenario_thread = threading.Thread(target=self.build_scenarios_loop, args=(self._debug_mode > 0, ))
        # self._scenario_thread.start()

        while self._running:
            self._tick_scenario()
        
    def _tick_scenario(self):
        """
        Run next tick of scenario and the agent and tick the world.
        """
        # if self._running and self.get_running_status():
        #     # very important!!!                                   
        #     CarlaDataProvider.get_world().tick(self._timeout)                

        timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()   
                                                                          
            self._watchdog.pause()

            if self.tick_count > self._tick_count_thd: 
                raise TickRuntimeError("RuntimeError, tick_count > {}".format(self._tick_count_thd))

            try:
                self._agent_watchdog.resume()
                self._agent_watchdog.update()

                #planning or collecting data in cur time 
                ego_action = self._agent_wrapper()

                self._agent_watchdog.pause()
            # Special exception inside the agent that isn't caused by the agent
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)
            
            self._watchdog.resume()
                                
            self.tick_count += 1  
            ## DriveE2E: tick ego actor
            if self.use_predefined_route:
                if self.tick_count < len(self.scenario.route):
                    # raise ValueError("Currently: The tick number must be less than 100.")                    
                    current_transform = self.scenario.ego_predefined_route[self.tick_count]                               
                    #need convert ego control
                    self.ego_vehicles[0].set_transform(current_transform)
                else:
                    self._running = False
            else:
                self.ego_vehicles[0].apply_control(ego_action)   
                                     
            ## DriveE2E: tick other actor
            if self._running and self.other_agent_setting:
                self._tick_other_actors()
                            
            # Tick scenario. Add the ego control to the blackboard in case some behaviors want to change it
            py_trees.blackboard.Blackboard().set("AV_control", ego_action, overwrite=True)
            self.scenario_tree.tick_once()
            
            if self._debug_mode > 1:
                self.compute_duration_time()

                # Update live statistics
                self._statistics_manager.compute_route_statistics(
                    self.route_index,
                    self.scenario_duration_system,
                    self.scenario_duration_game,
                    failure_message=""
                )
                self._statistics_manager.write_live_results(
                    self.route_index,
                    self.ego_vehicles[0].get_velocity().length(),
                    ego_action,
                    self.ego_vehicles[0].get_location()
                )

            if self._debug_mode > 2:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

            ego_trans = self.ego_vehicles[0].get_transform()
            self._spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=70),
                                                          carla.Rotation(pitch=-90)))
            
        if self._running and self.get_running_status():
            # very important!!!                                   
            CarlaDataProvider.get_world().tick(self._timeout)   

    def _tick_other_actors(self):        
        act_tick_count = self.tick_count
        
        for actor_id in list(self.scenario.other_routes_dict.keys()):
            
            # 跳过未到 spawn_time 的 actor
            if self.scenario.other_routes_dict[actor_id]['spawn_time'] > act_tick_count:
                continue

            # 跳过 destroy_time 已过的 actor
            if self.scenario.other_routes_dict[actor_id]['destroy_time'] + 1 < act_tick_count:
                continue

            # 在 spawn_time 时生成 actor
            if self.scenario.other_routes_dict[actor_id]['spawn_time'] == act_tick_count:
                blueprint_library = self.scenario.world.get_blueprint_library()
                vehicle_bp = blueprint_library.find(self.scenario.other_routes_dict[actor_id]['blueprint'])
                spawn_point = self.scenario.other_routes_dict[actor_id]['routes'][act_tick_count]
                try:
                    vehicle = self.scenario.world.spawn_actor(vehicle_bp, spawn_point)
                    #set init speed
                    target_speed = float(self.scenario.other_routes_dict[actor_id]['target_speed'][act_tick_count])
                    # print(f"actor_id: {actor_id} target_speed: {target_speed}")
                    if self.scenario.algo != "expert":
                        vehicle = self.scenario._set_init_speed(vehicle,target_speed)
                                                
                    if vehicle_bp.id.startswith('vehicle') and self.scenario.other_routes_dict[actor_id]['real_time'] > 18.5:
                        light_state = carla.VehicleLightState(carla.VehicleLightState.HighBeam |
                            carla.VehicleLightState.LowBeam |
                            carla.VehicleLightState.Position)
                        vehicle.set_light_state(light_state)
                    self.scenario.other_actors_dict[actor_id] = vehicle
                except Exception as e:
                    print(f"Failed to spawn actor {actor_id}: {e}")
                    del self.scenario.other_routes_dict[actor_id]
                    print(f"Removed actor_id {actor_id} from routes_dict")

            # 在 destroy_time 时销毁 actor
            elif self.scenario.other_routes_dict[actor_id]['destroy_time'] + 1 == act_tick_count:
                # 检查 actor 是否存在于字典中
                if actor_id in self.scenario.other_actors_dict:
                    self.scenario.other_actors_dict[actor_id].destroy()
                    del self.scenario.other_actors_dict[actor_id]  # 确保从字典中删除销毁的 actor

            # 更新 actor 的 transform
            else:
                routes = self.scenario.other_routes_dict[actor_id]['routes']
                
                # 确保 tick_count 在 routes 范围内
                if act_tick_count < len(routes):
                    current_transform = routes[act_tick_count]
                    
                    # 检查 current_transform 是否为 None
                    if current_transform is not None:
                        try:
                            self.scenario.other_actors_dict[actor_id].set_transform(current_transform)
                            if 'vehicle' in self.scenario.other_actors_dict[actor_id].type_id and self.scenario.other_routes_dict[actor_id]['real_time'] > 18.5:
                                lights_state = carla.VehicleLightState(carla.VehicleLightState.LowBeam |
                                                                        carla.VehicleLightState.Position |
                                                                        carla.VehicleLightState.HighBeam)
                                self.scenario.other_actors_dict[actor_id].set_light_state(lights_state)
                        except Exception as e:
                            print(f"Error setting transform for actor {actor_id}: {e}")
                    else:
                        print(f"Warning: current_transform is None for actor {actor_id}. Skipping transformation.")
                else:
                    print(f"Warning: tick_count {act_tick_count} is out of range for actor {actor_id}. Skipping transformation.")

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        if self._watchdog:
            return self._watchdog.get_status()
        return True

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        if self._watchdog:
            self._watchdog.stop()

        if self._agent_watchdog:
            self._agent_watchdog.stop()

        self.compute_duration_time()

        if self.get_running_status():
            if self.scenario is not None:
                self.scenario.terminate()

            if self._agent_wrapper is not None:
                self._agent_wrapper.cleanup()
                self._agent_wrapper = None

            self.analyze_scenario()

        # Make sure the scenario thread finishes to avoid blocks
        self._running = False
        # self._scenario_thread.join()
        # self._scenario_thread = None

    def compute_duration_time(self):
        """
        Computes system and game duration times
        """
        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        ResultOutputProvider(self)
