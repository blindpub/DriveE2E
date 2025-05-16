town_list="DriveE2ETown04Opt DriveE2ETown11Opt DriveE2ETown12Opt DriveE2ETown20Opt DriveE2ETown13Opt DriveE2ETown07Opt DriveE2ETown14Opt DriveE2ETown28Opt DriveE2ETown01Opt DriveE2ETown02Opt DriveE2ETown25Opt DriveE2ETown27Opt DriveE2ETown17Opt DriveE2ETown06Opt DriveE2ETown19Opt"

for town_name in $town_list; do
    echo ${town_name}
    python tools/gen_hdmap.py --carla_town "$town_name" --save_dir Bench2DriveZoo/data/drivee2e_maps_20241126
done