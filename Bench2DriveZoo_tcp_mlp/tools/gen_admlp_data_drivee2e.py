import os
import json
import numpy as np
from tqdm import trange
import gzip
import multiprocessing as mp 
import time
import re

INPUT_FRAMES = 5*5
FUTURE_FRAMES = 6*5

TRAIN = False


class Colors:
	RED = '\033[91m'
	GREEN = '\033[92m'
	YELLOW = '\033[93m'
	BLUE = '\033[94m'
	MAGENTA = '\033[95m'
	CYAN = '\033[96m'
	WHITE = '\033[97m'
	RESET = '\033[0m'

def gen_single_route(route_folder, count):

	folder_path = os.path.join(route_folder, 'anno')
	length = len([name for name in os.listdir(folder_path)]) - 1 # drop last frame

	if length < INPUT_FRAMES + FUTURE_FRAMES:
		return

	seq_future_x = []
	seq_future_y = []
	seq_future_theta = []

	seq_input_x = []
	seq_input_y = []
	seq_input_theta = []

	seq_input_speed = []
	seq_input_speed_acc = []
	seq_input_command = []

	full_seq_x = []
	full_seq_y = []
	full_seq_theta = []
	full_seq_speed = []
	full_seq_speed_acc = []
	full_seq_command = []

	for i in trange(length):
		with gzip.open(os.path.join(route_folder, f'anno/{i:05}.json.gz'), 'rt', encoding='utf-8') as gz_file:
			anno = json.load(gz_file)
		
		full_seq_x.append(anno['x'])
		full_seq_y.append(anno['y'])  # TODO(yzj): need to align sign
		full_seq_speed.append(anno['speed'])
		full_seq_speed_acc.append(anno['acceleration'])
		full_seq_theta.append(anno['theta'])
		full_seq_command.append(anno['next_command'])

	for i in trange(INPUT_FRAMES-5, length-FUTURE_FRAMES):
		with gzip.open(os.path.join(route_folder, f'anno/{i:05}.json.gz'), 'rt', encoding='utf-8') as gz_file:
			anno = json.load(gz_file)

		seq_input_x.append(full_seq_x[i-(INPUT_FRAMES-5):i+5:5])
		seq_input_y.append(full_seq_y[i-(INPUT_FRAMES-5):i+5:5])
		seq_input_theta.append(full_seq_theta[i-(INPUT_FRAMES-5):i+5:5])

		seq_input_speed.append(full_seq_speed[i-(INPUT_FRAMES-5):i+5:5])
		seq_input_speed_acc.append(full_seq_speed_acc[i-(INPUT_FRAMES-5):i+5:5])
		seq_input_command.append(full_seq_command[i-(INPUT_FRAMES-5):i+5:5])

		seq_future_x.append(full_seq_x[i+5:i+FUTURE_FRAMES+5:5])
		seq_future_y.append(full_seq_y[i+5:i+FUTURE_FRAMES+5:5])
		seq_future_theta.append(full_seq_theta[i+5:i+FUTURE_FRAMES+5:5])

	with count.get_lock():
		count.value += 1
	return seq_future_x, seq_future_y, seq_future_theta, seq_input_x, seq_input_y, seq_input_theta, seq_input_speed, seq_input_speed_acc, seq_input_command, full_seq_x, full_seq_y, full_seq_theta, full_seq_speed, full_seq_speed_acc, full_seq_command 

def gen_sub_folder(seq_data_list):
	print('begin saving...')
	total_future_x = []
	total_future_y = []
	total_future_theta = []

	total_input_x = []
	total_input_y = []
	total_input_theta = []

	total_input_speed = []
	total_input_speed_acc = []
	total_input_command = []

	for seq_data in seq_data_list:
		if not seq_data:
			continue
		seq_future_x, seq_future_y, seq_future_theta, seq_input_x, seq_input_y, seq_input_theta, seq_input_speed, seq_input_speed_acc, seq_input_command, full_seq_x, full_seq_y, full_seq_theta, full_seq_speed, full_seq_speed_acc, full_seq_command  = seq_data
		total_future_x.extend(seq_future_x)
		total_future_y.extend(seq_future_y)
		total_future_theta.extend(seq_future_theta)
		total_input_x.extend(seq_input_x)
		total_input_y.extend(seq_input_y)
		total_input_theta.extend(seq_input_theta)
		total_input_speed.extend(seq_input_speed)
		total_input_speed_acc.extend(seq_input_speed_acc)
		total_input_command.extend(seq_input_command)

	data_dict = {}
	data_dict['future_x'] = total_future_x
	data_dict['future_y'] = total_future_y
	data_dict['future_theta'] = total_future_theta
	data_dict['input_x'] = total_input_x
	data_dict['input_y'] = total_input_y
	data_dict['input_theta'] = total_input_theta
	data_dict['input_speed'] = total_input_speed
	data_dict['input_speed_acc'] = total_input_speed_acc
	data_dict['input_command'] = total_input_command		
	if TRAIN:
		file_path = os.path.join("admlp_drivee2e_final-train")
	else:
		file_path = os.path.join("admlp_drivee2e_final-val")
	np.save(file_path, data_dict)
	print(f'begin saving, length={len(total_future_x)}')

def get_folder_path(folder_paths, total, split):
	path = 'Bench2DriveZoo/data/drivee2e/final'
        for d0 in os.listdir(path):
		if TRAIN:
			if d0 in split['train']:
				folder_paths.put(os.path.join(path, d0))
				with total.get_lock():
					total.value += 1
		else:
			if d0 in split['val']:
				folder_paths.put(os.path.join(path, d0))
				with total.get_lock():
					total.value += 1
	return folder_paths

def worker(folder_paths, count, seq_data_list, stop_event, worker_num, completed_workers):
	while True:
		if folder_paths.qsize()<=0:
			with completed_workers.get_lock():
				completed_workers.value += 1
				if completed_workers.value == worker_num:
					stop_event.set()
			break
		folder_path = folder_paths.get()
		seq_data = gen_single_route(folder_path, count)
		seq_data_list.append(seq_data)

def display(count, total, stop_event, completed_workers):
	t1 = time.time()
	while True:
		print(f'{Colors.GREEN}[count/total]=[{count.value}/{total.value}, {count.value/(time.time()-t1):.2f}it/s, completed_workers={completed_workers.value}]{Colors.RESET}', flush=True)
		time.sleep(3)
		if stop_event.is_set():
			break

def process_json(path_json):
    with open(path_json, "r") as load_f:
        my_json = json.load(load_f)

    ret_dict = {
        'train': [],
        'val': [],
    }
    to_remove = 'final/'
    for set_name, set_list in my_json.items():
        if set_name in ret_dict:  # 确保set_name是期望的键
            for scene in set_list:
                m_scene = re.sub(to_remove, '', scene)
                ret_dict[set_name].append(m_scene)
    return ret_dict

if __name__ == '__main__':
	folder_paths = mp.Queue()
	seq_data_list = mp.Manager().list()
	count = mp.Value('d', 0)
	total = mp.Value('d', 0)
	stop_event = mp.Event()
	completed_workers = mp.Value('d', 0)

	split = process_json('tools/drivee2e_train_val_split.json')

	get_folder_path(folder_paths, total, split)
	ps = []
	worker_num = 64
	for i in range(worker_num):
		p = mp.Process(target=worker, args=(folder_paths, count, seq_data_list, stop_event, worker_num, completed_workers, ))
		p.daemon = True
		p.start()
		ps.append(p)
	
	p = mp.Process(target=display, args=(count, total, stop_event, completed_workers))
	p.daemon = True
	p.start()
	ps.append(p)
	
	for p in ps:
		p.join()
	
	display(count, total, stop_event, completed_workers)
	gen_sub_folder(seq_data_list)
	display(count, total, stop_event, completed_workers)
