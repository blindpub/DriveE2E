import carla
import pyproj
import pandas as pd
import pymap3d
import math
from datetime import datetime, timezone, timedelta

## DriveE2E
def obj_speed(vx, vy):
    return math.sqrt(vx**2 + vy**2)

def origin_lat_lon(world_name):
    world_dir = {
        'DriveE2ETown04':[39.77915851408599, 116.5108787076835],
        'DriveE2ETown04NoUnderline':[39.77915851408599, 116.5108787076835],
        'DriveE2ETown04Opt':[39.77915851408599, 116.5108787076835],
        'DriveE2ETown07':[39.78489640198514, 116.5247559414074],
        'DriveE2ETown11':[39.76666775559748, 116.511681624311],
        'DriveE2ETown12':[39.76921740297173, 116.5093496370261],
        'DriveE2ETown13':[39.77284732654662, 116.5054455692828],
        'DriveE2ETown14':[39.78215153851394, 116.4990152653169],
        'DriveE2ETown20':[39.7742631876477, 116.5097392323581],
        'DriveE2ETown28':[39.77608874527081, 116.5137718382781],
        'DriveE2ETown01':[39.77926323783298, 116.5011103243239],
        'DriveE2ETown02':[39.77631595960691, 116.503948792419],
        'DriveE2ETown06':[39.78255066071395, 116.5196464234889],
        'DriveE2ETown17':[39.79196422380497, 116.4902554261707],
        'DriveE2ETown19':[39.77099668165076, 116.5121942659732],
        'DriveE2ETown25':[39.7715713941659, 116.4960274192687],
        'DriveE2ETown27':[39.76315776526656, 116.4780709064877],
        'DriveE2ETown04Opt':[39.77900053316968, 116.5109633667903],
        'DriveE2ETown07Opt':[39.78489640198514, 116.5247559414074],
        'DriveE2ETown11Opt':[39.76666775559748, 116.511681624311],
        'DriveE2ETown12Opt':[39.76921740297173, 116.5093496370261],
        'DriveE2ETown13Opt':[39.77284732654662, 116.5054455692828],
        'DriveE2ETown14Opt':[39.78215153851394, 116.4990152653169],
        'DriveE2ETown20Opt':[39.7742631876477, 116.5097392323581],
        'DriveE2ETown28Opt':[39.77608874527081, 116.5137718382781],
        'DriveE2ETown01Opt':[39.77926323783298, 116.5011103243239],
        'DriveE2ETown02Opt':[39.77631595960691, 116.503948792419],
        'DriveE2ETown06Opt':[39.78255066071395, 116.5196464234889],
        'DriveE2ETown17Opt':[39.79196422380497, 116.4902554261707],
        'DriveE2ETown19Opt':[39.77099668165076, 116.5121942659732],
        'DriveE2ETown25Opt':[39.7715713941659, 116.4960274192687],
        'DriveE2ETown27Opt':[39.76315776526656, 116.4780709064877],
    }

    return world_dir[world_name]

def modi_origin_lat_lon(tmp_lat, tmp_lon):
    lat_dis = 0.01584378096442407
    lon_dis = -0.0002900562694776454
    
    lat_rst = tmp_lat + lat_dis
    lon_rst = tmp_lon + lon_dis

    return lat_rst, lon_rst
    

def cs_transform(x1, y1, town_name):
    ref_x, ref_y = -40251.76572214719, 326531.9706723457

    x_tmp, y_tmp = x1 - ref_x, y1 - ref_y
    p1 = pyproj.Proj("+proj=utm +lat_0=0 +lon_0=117 +zone=50 +k=1 +x_0=500000 +y_0=0 +unit=m +type=crs", preserve_units=False)
    car_real_lon, car_real_lat = p1(x_tmp, y_tmp, inverse=True)
    car_real_alt = 0

    tmp_lat, tmp_lon = origin_lat_lon(town_name)
    origin_lat, origin_lon = modi_origin_lat_lon(tmp_lat, tmp_lon)
    # origin_lon = 116.51058865141403
    # origin_lat = 39.795002295050416
    origin_alt = 0
    x_rst, y_rst, z_rst = pymap3d.geodetic2enu(car_real_lat, car_real_lon, car_real_alt, origin_lat, origin_lon, origin_alt)
    y_rst = -y_rst

    x_bias = 1.85
    y_bias = 3.20429
    x_rst += x_bias
    y_rst += y_bias

    return x_rst, y_rst 

def get_drivee2e_routes(predefined_route_path, world, town_name, frame_rate):

    # step 1: load data
    ego_value = int(predefined_route_path.split('_')[-1].split('.')[0])
    raw_data = pd.read_csv(predefined_route_path)
    colums_we_want = ['timestamp','id','type','sub_type', 'x','y', 'length','width','height', 'theta', 'v_x', 'v_y', 'blueprint'] 
    raw_data = raw_data[colums_we_want]
    processed_data = raw_data
    N_frames = len(processed_data.loc[processed_data['id'] == ego_value, "timestamp"].values)

    # step 2: obtain the duration of each object
    all_ids_that_we_have = processed_data.id.unique()
    human_id_list = []

    pedestrian_df = processed_data[processed_data['type'] == 'PEDESTRIAN']

    # 提取唯一的id并转换为列表
    pedestrian_id_list =pedestrian_df['id'].unique().tolist()

    actor_spawn_time = {}
    actor_destroy_time = {}
    whole_scenario_stamp_min, whole_scenario_stamp_max = min(processed_data.timestamp), max(processed_data.timestamp)

    time = datetime.fromtimestamp(whole_scenario_stamp_min, tz=timezone.utc)
    beijing_time = time + timedelta(hours=8)
    beijing_fractional_hours = beijing_time.hour + beijing_time.minute / 60 + beijing_time.second / 3600

    for each_id in all_ids_that_we_have:
        all_row_of_each_id = processed_data.loc[processed_data.id == each_id]
        t_min, t_max = min(all_row_of_each_id.timestamp), max(all_row_of_each_id.timestamp)
        actor_spawn_time[each_id], actor_destroy_time[each_id] = t_min - whole_scenario_stamp_min, \
            t_max - whole_scenario_stamp_min
    
    check_destroy_time = {}
    for i in actor_destroy_time:
        check_destroy_time[i] = round(actor_destroy_time[i]*10)

    check_spawn_time = {}
    for i in actor_spawn_time:
        check_spawn_time[i] = round(actor_spawn_time[i]*10)

    all_routes_dict = {}

    for this_specific_id in all_ids_that_we_have:
        all_data_of_this_id = processed_data.loc[processed_data.id == this_specific_id].reset_index(drop=True)

        wps = [None] * N_frames
        speed = [None] * N_frames
        velocity = [None] * N_frames
        spawn_time = check_spawn_time[this_specific_id]
        
        for this_frame in range(len(all_data_of_this_id)): 
            x_, y_ = cs_transform(all_data_of_this_id.x.values[this_frame], all_data_of_this_id.y.values[this_frame], town_name)
            
            if this_specific_id in pedestrian_id_list:
                if world is None:
                    # z_ = all_data_of_this_id.height.values[this_frame] / 2 # Need to change to new height according to the assigned
                    z_ = 0.6
                else:
                    # z_ = get_z(world, x_, y_)
                    pass
            
            else:
                if world is None:
                    # z_ = all_data_of_this_id.height.values[this_frame] / 2 # Need to change to new height according to the assigned
                    z_ = 0.2
                else:
                    # z_ = get_z(world, x_, y_)
                    pass
            location = carla.Location(x=x_, y=y_, z=z_)
            rotation = carla.Rotation(pitch=0.0, yaw=-all_data_of_this_id.theta.values[this_frame]*180/math.pi, roll=0.0)
            transform = carla.Transform(location, rotation)
            wps[this_frame + spawn_time] = transform
            
            tmp_vx = all_data_of_this_id.v_x.values[this_frame]
            tmp_vy = all_data_of_this_id.v_y.values[this_frame]
            obj_speed_this_frame = obj_speed(tmp_vx, tmp_vy)
            speed[this_frame + spawn_time] = obj_speed_this_frame
            velocity[this_frame + spawn_time] = (tmp_vx, tmp_vy)

        all_routes_dict[this_specific_id] = {}
        all_routes_dict[this_specific_id]['routes'] = wps
        all_routes_dict[this_specific_id]['target_speed'] = speed
        all_routes_dict[this_specific_id]['velocity'] = velocity
        all_routes_dict[this_specific_id]['spawn_time'] = check_spawn_time[this_specific_id]
        all_routes_dict[this_specific_id]['destroy_time'] = check_destroy_time[this_specific_id]
        all_routes_dict[this_specific_id]['type'] = all_data_of_this_id.type.values[this_frame]
        all_routes_dict[this_specific_id]['sub_type'] = all_data_of_this_id.sub_type.values[0]
        all_routes_dict[this_specific_id]['real_time'] = beijing_fractional_hours
        all_routes_dict[this_specific_id]['blueprint'] = all_data_of_this_id.blueprint.values[0]         

    return all_routes_dict

def get_ego_route(predefined_route_path, ego_vehicle_id):
    all_routes_dict = get_drivee2e_routes(predefined_route_path)

    return all_routes_dict[ego_vehicle_id]['routes']


def get_traffic_light_dir(traffic_light_path):
    '''
    return dir.
    key: traffic light actor
    value: Command in time series
    '''
    # pd.read_csv(traffic_light_path)
    pass


if __name__ == "__main__":
    # predefined_route_path = '10.csv'
    # ego_vehicle_id = 200001
    # all_routes_dict = get_drivee2e_routes(predefined_route_path)
    # ego_route = get_ego_route(predefined_route_path, ego_vehicle_id)
    # client = carla.Client('127.0.0.1',2008)
    # world = client.get_world()
    # cs_transform(0,0,'DriveE2ETown04')
    tmplat, tmplon = origin_lat_lon('DriveE2ETown04')
    lat, lon = modi_origin_lat_lon(tmplat, tmplon)
    print(lat, lon)
    pass
