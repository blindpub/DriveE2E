import json
import os
from pathlib import Path

def get_two_level_folder_names(data_root, DATA_VERSION):
    folder_names = []

    root_dir = os.path.join(data_root, DATA_VERSION)
    for root, dirs, files in os.walk(root_dir):
        level = root.count(os.sep) - root_dir.count(os.sep)
        
        if level == 0:
            relative_dirs = [os.path.relpath(os.path.join(root, d), data_root) for d in dirs]
            folder_names.extend(relative_dirs)

        if level >= 1:
            del dirs[:]
    
    return folder_names

def split_list_by_ratio(input_list):
    list1, list2 = [], []
    i = 0
    while i < len(input_list):
        list1.extend(input_list[i:i+2])
        i += 2
        list2.extend(input_list[i:i+1])
        i += 1
    
    return list1, list2

def generate_split(data_root, DATA_VERSION, save_split_path):
    folder_names = get_two_level_folder_names(data_root, DATA_VERSION)
    train_list, val_list = split_list_by_ratio(folder_names)

    split_file = {
        "train": train_list,
        "val": val_list
    }

    print(len(train_list))
    print(len(val_list))

    with open(save_split_path, 'w') as json_file:
        json.dump(split_file, json_file, indent=4, ensure_ascii=False)


if __name__ == "__main__":
    data_root = "Bench2DriveZoo/data/drivee2e"
    DATA_VERSION = 'final'

    save_split_path = "../../data/splits/drivee2e_train_val_split_new.json"
    generate_split(data_root, DATA_VERSION, save_split_path)
