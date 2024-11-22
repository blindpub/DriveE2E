import json
import carla
import argparse
import xml.etree.ElementTree as ET
from agents.navigation.global_route_planner import GlobalRoutePlanner
import os,sys,glob
import atexit
import subprocess
import time
import random

Ability = ["COV", "IPC", "UT", "YLW","STR", "LFT", "RT", "STP"]

Ability_nums = {"RT": 21,
                "STR": 33,
                "LFT": 23,
                "STP": 18,
                "YLW": 15,
                "COV": 32,
                "IPC": 28,
                "UT":5}


def get_infraction_status(record):
    for infraction,  value in record['infractions'].items():
        if infraction == "min_speed_infractions":
            continue
        elif len(value) > 0:
            return True
    return False

def update_Ability(scenario_name, Ability_Statistic, status):
    # for ability, scenarios in Ability.items():
    for scenarios in Ability:
        if scenario_name in scenarios:
            Ability_Statistic[scenario_name][1] += 1
            if status:
                Ability_Statistic[scenario_name][0] += 1
    pass

def update_Success(scenario_name, Success_Statistic, status):
    if scenario_name not in Success_Statistic:
        if status:
            Success_Statistic[scenario_name] = [1, 1]
        else:
            Success_Statistic[scenario_name] = [0, 1]
    else:
        Success_Statistic[scenario_name][1] += 1
        if status:
            Success_Statistic[scenario_name][0] += 1
    pass

def get_position(xml_route):
    waypoints_elem = xml_route.find('waypoints')
    keypoints = waypoints_elem.findall('position')
    return [carla.Location(float(pos.get('x')), float(pos.get('y')), float(pos.get('z'))) for pos in keypoints]

def get_route_result(records, route_id):
    for record in records:
        record_route_id = record['route_id'].split('_')[1]
        if route_id == record_route_id:
            return record
    return None

def get_waypoint_route(locs, grp):
    route = []
    for i in range(len(locs) - 1):
        loc = locs[i]
        loc_next = locs[i + 1]
        interpolated_trace = grp.trace_route(loc, loc_next)
        for wp, _ in interpolated_trace:
            route.append(wp)
    return route

def main(args):
    routes_path = args.xml_file_path 
    result_file = args.result_file
    Ability_Statistic = {}
    crash_route_list = []
    for key in Ability:
        Ability_Statistic[key] = [0, 0.]
    Success_Statistic = {}
    
    with open(result_file, 'r') as f:
        data = json.load(f)
    records = data["_checkpoint"]["records"]
    
    file_paths = glob.glob(f'{routes_path}/*.xml')
    total_routes = []
    for routes_file in file_paths:                    
        tree = ET.parse(routes_file)
        root = tree.getroot()
        routes = root.findall('route')
        total_routes.extend(routes)
        
    routes = total_routes
    sorted_routes = sorted(routes, key=lambda x: x.get('town'))
    routes_nums = len(routes)
    
    for route in sorted_routes:
        # scenarios = route.find('scenarios')
        # scenario_name = scenarios.find('scenario').get("type")
        
        scenario_csv_id = route.get('scenario_csv_id') #"YLW_000_200027"
        scenario_name = scenario_csv_id.split('_')[0]

        route_id = route.get('id')
        route_record = get_route_result(records, route_id)
        if route_record is None:
            crash_route_list.append((scenario_name, route_id))
            print('No result record of route', route_id, "in the result file")
            continue
        if route_record["status"] == 'Completed' or route_record["status"] == "Perfect":
            if get_infraction_status(route_record):
                record_success_status = False
            else:
                record_success_status = True
        else:
            record_success_status = False
        update_Ability(scenario_name, Ability_Statistic, record_success_status)
        update_Success(scenario_name, Success_Statistic, record_success_status)
        # if scenario_name in Ability["Traffic_Signs"] and (scenario_name in Ability["Merging"] or scenario_name in Ability["Emergency_Brake"]):
        # Only these three 'Ability's intersect
        
    Ability_Res = {}
    for ability, statis in Ability_Statistic.items():
        if statis[1] != 0:
            # Ability_Res[ability] = float(statis[0])/float(statis[1]) * 100
            Ability_Res[ability] = float(statis[0])/Ability_nums[ability] * 100
        else:
            Ability_Res[ability] = 0
        
    for key, value in Ability_Res.items():
        print(key, ": ", value)
    
    Ability_Res['mean'] = sum(list(Ability_Res.values())) / len(Ability_Res)
    print("Mean: ", Ability_Res['mean'] )
    Ability_Res['crashed'] = crash_route_list
    with open(f"{result_file.split('.')[0]}_ability.json", 'w') as file:
        json.dump(Ability_Res, file, indent=4)
        
    Success_Res = {}
    Route_num = 0
    Succ_Route_num = 0
    for scenario, statis in Success_Statistic.items():
        Success_Res[scenario] = float(statis[0])/float(statis[1])
        Succ_Route_num += statis[0]
        Route_num += statis[1]
    assert len(crash_route_list) == routes_nums - float(Route_num)
    # print(f'Crashed Route num: {len(crash_route_list)}, Crashed Route ID: {crash_route_list}')
    print(f'(Record Route num + Crashed Route num) = Total Route num: ({Route_num} + {len(crash_route_list)}) = {routes_nums} ')
    
    print('Finished!')

if __name__=='__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('-x', '--xml_file_path', nargs=None, default="/data_storage/ad_sharing/datasets/Trajectory-Forecasting-Dataset/drivee2e_data/drive_e2e_xmls_val_new/", help='xml_file_path')
    argparser.add_argument('-r', '--result_file', nargs=None, default="", help='result json file')
        
    args = argparser.parse_args()
        
    main(args)
    print(f"xml_file_path: {args.xml_file_path}; result_file: {args.result_file}")
    
