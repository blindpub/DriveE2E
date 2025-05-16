#!/bin/bash
export CARLA_ROOT=../carla
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:leaderboard
export PYTHONPATH=$PYTHONPATH:leaderboard/team_code
export PYTHONPATH=$PYTHONPATH:scenario_runner
export SCENARIO_RUNNER_ROOT=scenario_runner

export LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS
export PORT=$1
export TM_PORT=$2
export DEBUG_CHALLENGE=0
export REPETITIONS=1 # multiple evaluation runs
export RESUME=False
export IS_BENCH2DRIVE=$3
export PLANNER_TYPE=$9
export GPU_RANK=${10}

# TCP evaluation
export ROUTES=$4
export TEAM_AGENT=$5
export TEAM_CONFIG=$6
export CHECKPOINT_ENDPOINT=$7
export SAVE_PATH=$8

# DriveE2E
USE_PREDEFINED_ROUTE=${11}
if [ -z "$USE_PREDEFINED_ROUTE" ]; then
    USE_PREDEFINED_ROUTE=False
fi

echo "USE_PREDEFINED_ROUTE = "$USE_PREDEFINED_ROUTE
OTHER_AGENT_SETTING=${12}
if [ -z "$OTHER_AGENT_SETTING" ]; then
    OTHER_AGENT_SETTING=False
fi

DRIVEE2E_SCENARIO_ROOT=${13}
if [ -z "$DRIVEE2E_SCENARIO_ROOT" ]; then
    DRIVEE2E_SCENARIO_ROOT=''
fi

ROUTEWEATHER_FILE=${14}
WEATHER_FILE=${15}
SCENARIO_FLAG=${16}
SIM_TIME_THD=${17}
ALGO=${18}

echo "GPU_RANK:"${GPU_RANK}
CUDA_VISIBLE_DEVICES=${GPU_RANK} python ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--agent-config=${TEAM_CONFIG} \
--debug=${DEBUG_CHALLENGE} \
--record=${RECORD_PATH} \
--resume=${RESUME} \
--port=${PORT} \
--traffic-manager-port=${TM_PORT} \
--gpu-rank=${GPU_RANK} \
--use-predefined-route=${USE_PREDEFINED_ROUTE} \
--other-agent-setting ${OTHER_AGENT_SETTING} \
--drivee2e-scenario-root ${DRIVEE2E_SCENARIO_ROOT} \
--route2weather-file=${ROUTEWEATHER_FILE} \
--weather-file=${WEATHER_FILE} \
--scenario-flag=${SCENARIO_FLAG} \
--simulation-time-thd=${SIM_TIME_THD} \
--algo=${ALGO}

