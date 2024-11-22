import json
import glob
import argparse
import os

def merge_route_json(folder_path):
    file_paths = glob.glob(f'{folder_path}/*eval*.json')
    merged_records = []
    driving_score = []
    success_num = 0
    records_nums = 0
    for file_path in file_paths:
        if 'merged.json' in file_path: continue
        with open(file_path) as file:
            data = json.load(file)
            records = data['_checkpoint']['records']
            records_nums += len(records) 
            for rd in records:
                rd.pop('index')
                merged_records.append(rd)
                driving_score.append(rd['scores']['score_composed'])
                if rd['status']=='Completed' or rd['status']=='Perfect':
                    success_flag = True
                    for k,v in rd['infractions'].items():
                        if len(v)>0 and k != 'min_speed_infractions':
                            success_flag = False
                            break
                    if success_flag:
                        success_num += 1
                        print("success: ", rd['route_id'])

    if len(merged_records) != args.route_nums:
        print(f"-----------------------Warning: there are {len(file_paths)} json files, {len(merged_records)} routes in your json, which does not equal to {args.route_nums}. All metrics (Driving Score, Success Rate, Ability) are inaccurate!!!")

    merged_records = sorted(merged_records, key=lambda d: d['route_id'], reverse=True)
    _checkpoint = {
        "records": merged_records
    }
    
    if records_nums > 0:
        merged_data = {
            "_checkpoint": _checkpoint,
            "runed_route_num": len(file_paths),
            "driving score": sum(driving_score) / args.route_nums,
            "success rate": success_num / args.route_nums * 100,
            "eval num": len(driving_score),
            "success num": success_num
        }
    else:
        merged_data = {
            "_checkpoint": _checkpoint,
            "runed_route_num": len(file_paths),
            "driving score": 0.0,
            "success rate": 0.0,
            "eval num": len(driving_score),
            "success num": 0
        }   
        
    print(f"driving score: {merged_data['driving score']}\nsuccess rate: {merged_data['success rate']}")     

    with open(os.path.join(folder_path, 'merged.json'), 'w') as file:
        json.dump(merged_data, file, indent=4)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--folder', help='old foo help',default="")
    parser.add_argument('-n', '--route-nums', default=175, type=float,
                        help='total route nums')   
        
    args = parser.parse_args()
    
    # args.folder = "Bench2Drive/batch_close_loop_test"
    # args.route_nums = 175 #600
        
    merge_route_json(args.folder)
    print(f"json results path: {args.folder}, route_nums: {args.route_nums} ")
