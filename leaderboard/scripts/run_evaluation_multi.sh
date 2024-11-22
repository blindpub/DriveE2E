#!/bin/bash
BASE_PORT=1068
BASE_TM_PORT=23174
SCENARIO_FLAG="DriveE2E"

GPU_IDS=(2 3 4 5)
ALGO=expert
DATA_VERSION=V1120
PLANNER_TYPE=traj
#PLANNER_TYPE=merge_ctrl_traj

#clip file path
DRIVEE2E_SCENARIO_ROOT="leaderboard/data/drive_e2e_clip_data_examples"
#xml file
directory="leaderboard/data/drive_e2e_xmls_val_examples"

# save path for alg
SAVE_PATH=./data_drivee2e_${ALGO}_${PLANNER_TYPE}_${DATA_VERSION}
# SAVE_PATH=None
echo "SAVE_PATH: "$SAVE_PATH

# TEAM_AGENT=leaderboard/team_code/your_team_agent.py
# TEAM_CONFIG=your_team_agent_ckpt.pth   # for TCP and ADMLP
# TEAM_CONFIG=your_team_agent_config.py+your_team_agent_ckpt.pth # for UniAD and VAD
if [ "$ALGO" == "uniad" ]; then
    USE_PREDEFINED_ROUTE=False
    TEAM_AGENT=Bench2DriveZoo/team_code/uniad_b2d_agent.py
    TEAM_CONFIG=Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/base_e2e_b2d.py+Bench2DriveZoo/adzoo/uniad/work_dirs/stage2_e2e/base_e2e_b2d/epoch_3.pth    
elif [ "$ALGO" == "vad" ]; then
    USE_PREDEFINED_ROUTE=False
    TEAM_AGENT=Bench2DriveZoo/team_code/vad_b2d_agent_visualize.py
    TEAM_CONFIG=Bench2DriveZoo/adzoo/vad/drivee2e_model/1001/VAD_base_e2e_b2d_20240929.py+Bench2DriveZoo/adzoo/vad/drivee2e_model/1001/epoch_2.pth
elif [ "$ALGO" == "expert" ]; then
    USE_PREDEFINED_ROUTE=True 
    TEAM_AGENT=tools/data_collect.py
    TEAM_CONFIG="None"    
elif [ "$ALGO" == "mlp" ]; then
    USE_PREDEFINED_ROUTE=False
    TEAM_AGENT=Bench2DriveZoo_tcp_mlp/team_code/admlp_b2d_agent.py
    TEAM_CONFIG=Bench2DriveZoo_tcp_mlp/adzoo/ad_mlp/last-v3.ckpt
elif [ "$ALGO" == "tcp" ]; then
    USE_PREDEFINED_ROUTE=False
    TEAM_AGENT=Bench2DriveZoo_tcp_mlp/team_code/tcp_b2d_agent.py
    TEAM_CONFIG=Bench2DriveZoo_tcp_mlp/adzoo/tcp/last-v2.ckpt
else
    echo "Unknown ALGO value: $ALGO"
fi

echo "TEAM_AGENT is set to: $TEAM_AGENT"

#common paras
SIM_TIME_THD=12
OTHER_AGENT_SETTING=True
IS_BENCH2DRIVE=True
BASE_CHECKPOINT_ENDPOINT=eval

#other paras
ROUTEWEATHER_FILE="leaderboard/data/route2weather.json"
WEATHER_FILE="leaderboard/data/weather.xml"

#all xml
xml_files=($(ls "$directory"/drive_e2e_*.xml))
total_files=${#xml_files[@]}
num_gpus=${#GPU_IDS[@]}

echo "total_files:"$total_files", num_gpus: "$num_gpus

# each nums
files_per_gpu=$(( (total_files + num_gpus - 1) / num_gpus ))

# Start an independent background task for each GPU.
for ((i = 0; i < num_gpus; i++)); do
  gpu_id=${GPU_IDS[i]}
  
  PORT=$((BASE_PORT + i * 150))
  TM_PORT=$((BASE_TM_PORT + i * 150))

  echo "gpu_id: "$gpu_id", PORT: "$PORT", TM_PORT: "$TM_PORT

  # Calculate the file range assigned to the GPU.
  start_index=$(( i * files_per_gpu ))
  end_index=$(( start_index + files_per_gpu - 1 ))

  # Ensure the file array length is not exceeded. 
  if [ "$end_index" -ge "$total_files" ]; then
    end_index=$(( total_files - 1 ))
  fi

  # A background task for the GPU to sequentially process the assigned XML files.
  (
    for ((j = start_index; j <= end_index; j++)); do
      xml_file=${xml_files[j]}
      filename=$(basename -- "$xml_file")
      number=${filename#"drive_e2e_"}
      number=${number%.xml}

      echo "Processing $xml_file on GPU $gpu_id"

      CHECKPOINT_ENDPOINT="${ALGO}_${BASE_CHECKPOINT_ENDPOINT}_$number.json"
      echo "cur CHECKPOINT_ENDPOINT: "$CHECKPOINT_ENDPOINT
      rm ${CHECKPOINT_ENDPOINT}

      bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $xml_file $TEAM_AGENT $TEAM_CONFIG  $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE ${gpu_id} ${USE_PREDEFINED_ROUTE} ${OTHER_AGENT_SETTING} ${DRIVEE2E_SCENARIO_ROOT} ${ROUTEWEATHER_FILE} ${WEATHER_FILE} ${SCENARIO_FLAG} ${SIM_TIME_THD} ${ALGO}
      echo "end processing: $xml_file"
      sleep 10
    done
  ) &

done
