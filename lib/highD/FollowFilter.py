
import pandas as pd
from Filter import Filter
import numpy as np

class FollowFilter():

    def filter_vehicle_follow_scenario(dataframe, 
                                       ego_type, preceding_type, 
                                       minDuration=None):
        print('Filtering vehicle follow scenario', ego_type, preceding_type, minDuration)
        def set_dict():
            return {
                'dataset_id': [], 'scenario_id': [], 
                'ego_id': [], 'preceding_id': [], 
                'frame': [], 
                'ego_x': [], 'ego_y': [], 
                'ego_vx': [], 'ego_vy': [], 
                'ego_ax': [], 'ego_ay': [],
                'preceding_x': [], 'preceding_y': [], 
                'preceding_vx': [], 'preceding_vy': [],
                'preceding_ax': [], 'preceding_ay': [],
            }
        fps = 25
        # group dataframe by dataset_id
        grouped = dataframe.groupby('dataset_id')
        df_list = []
        scenario_id = 1
        for dataset_id, group in grouped:
            car_following_dict = set_dict()
            df = group.copy()
            # filter by agent type and lane changes
            fActor = Filter._filter_actors(df, 'class', ego_type, 'nLane', 1)
            pActor = Filter._filter_actors(df, 'class', preceding_type, 'nLane', 1)
            
            for actor in fActor:
                actor_df = df[df['id'] == actor]
                preceding_id = list(set(actor_df['precedingId'].unique()) - {0}) # by default there is always 0                
                # check if the preceding agent is of filtered class and lane change number
                filtered_preceding_id = np.intersect1d(preceding_id, pActor) # intersection of two arrays
                for p_id in filtered_preceding_id:
                    # check if the preceding agent has any vehicle at front
                    preceding_agent_df = df[df['id'] == p_id]
                    preceding_preceding_id = preceding_agent_df['precedingId'].unique() 
                    if len(preceding_preceding_id) > 1:
                        # print('preceding agent has a preceding agent, skipping')
                        continue
                    frames_togather = actor_df[actor_df['precedingId'] == p_id]
                    if minDuration is not None:
                        if len(frames_togather) < minDuration * fps:
                            # print('scenario duration is less than minDuration, skipping')
                            continue
                    ego_tracks = df[(df['id'] == actor) & (df['frame'].isin(frames_togather['frame']))]
                    preceding_tracks = df[(df['id'] == p_id) & (df['frame'].isin(frames_togather['frame']))]
                    
                    # create a dataframe with ego and preceding agent's data
                    car_following_dict['scenario_id'] = [scenario_id] * len(frames_togather)
                    car_following_dict['dataset_id'] = [ego_tracks['dataset_id'].values[0]] * len(frames_togather) 
                    car_following_dict['ego_id'] = [actor] * len(frames_togather) 
                    car_following_dict['preceding_id'] = [p_id] * len(frames_togather)
                    
                    car_following_dict['frame'] = frames_togather['frame'].values - frames_togather['frame'].values[0] # so that the first frame is 0
                    
                    car_following_dict['ego_x'] = ego_tracks['x'].values
                    car_following_dict['ego_y'] = ego_tracks['y'].values
                    car_following_dict['ego_vx'] = ego_tracks['xVelocity'].values
                    car_following_dict['ego_vy'] = ego_tracks['yVelocity'].values
                    car_following_dict['ego_ax'] = ego_tracks['xAcceleration'].values
                    car_following_dict['ego_ay'] = ego_tracks['yAcceleration'].values
                    
                    car_following_dict['preceding_x'] = preceding_tracks['x'].values
                    car_following_dict['preceding_y'] = preceding_tracks['y'].values
                    car_following_dict['preceding_vx'] = preceding_tracks['xVelocity'].values
                    car_following_dict['preceding_vy'] = preceding_tracks['yVelocity'].values
                    car_following_dict['preceding_ax'] = preceding_tracks['xAcceleration'].values
                    car_following_dict['preceding_ay'] = preceding_tracks['yAcceleration'].values
                    
                    scenario_id += 1
                    temp_df = pd.DataFrame.from_dict(car_following_dict)
                    df_list.append(temp_df)
                    # print('scenario added to df_list')
                    pass
                pass
            pass
        
        df = pd.concat(df_list)
        return df
    
    # this function first remove scenarios with high vy oscillation
    # then it translates the scenario wrt initial (ego_x, ego_y)
    # than remove scenarios 
    def process_vehicle_follow_scenario(follow_meta, yOscillationRange=None, maxDistance=None):
        follow_meta_copy = follow_meta.copy()
        if yOscillationRange is not None:
            low, high = yOscillationRange
            follow_meta_copy = FollowFilter.remove_vy_oscillation_from_follow_meta(follow_meta_copy, low, high)
        follow_meta_copy = FollowFilter.translate_follow_meta_wrt_ego_xy(follow_meta_copy)
        if maxDistance is not None:
            follow_meta_copy = FollowFilter.filter_follow_meta_by_initial_distance(follow_meta_copy, maxDistance)
        return follow_meta_copy
    
    def remove_vy_oscillation_from_follow_meta(follow_meta, low, high):
        new_follow_meta = follow_meta.copy()
        for scenario in new_follow_meta.scenario_id.unique():
            df = new_follow_meta[new_follow_meta['scenario_id'] == scenario]
            ego_low_vy = df['ego_vy'].min()
            ego_high_vy = df['ego_vy'].max()
            preceding_low_vy = df['preceding_vy'].min()
            preceding_high_vy = df['preceding_vy'].max()
            if preceding_low_vy < low or preceding_high_vy > high:
                follow_meta = follow_meta[follow_meta['scenario_id'] != scenario]
            if ego_low_vy < low or ego_high_vy > high:
                follow_meta = follow_meta[follow_meta['scenario_id'] != scenario]
        return follow_meta
    
    # this function also changes the velocity and acceleration to absolute values
    # to make sure that the ego vehicle is always in the positive x-axis direction
    # so we can use the same setup for all scenarios
    def translate_follow_meta_wrt_ego_xy(follow_meta):
        for scenario in follow_meta.scenario_id.unique():
            df = follow_meta[follow_meta['scenario_id'] == scenario]
            first_x = df['ego_x'].iloc[0]
            first_y = df['ego_y'].iloc[0]
            follow_meta.loc[follow_meta['scenario_id'] == scenario, 'ego_x'] = follow_meta.loc[follow_meta['scenario_id'] == scenario, 'ego_x'] - first_x
            follow_meta.loc[follow_meta['scenario_id'] == scenario, 'ego_y'] = follow_meta.loc[follow_meta['scenario_id'] == scenario, 'ego_y'] - first_y
            follow_meta.loc[follow_meta['scenario_id'] == scenario, 'preceding_x'] = follow_meta.loc[follow_meta['scenario_id'] == scenario, 'preceding_x'] - first_x
            follow_meta.loc[follow_meta['scenario_id'] == scenario, 'preceding_y'] = follow_meta.loc[follow_meta['scenario_id'] == scenario, 'preceding_y'] - first_y
        
        follow_meta['ego_x'] = abs(follow_meta['ego_x'])
        follow_meta['preceding_x'] = abs(follow_meta['preceding_x'])

        follow_meta['ego_vx'] = abs(follow_meta['ego_vx'])
        follow_meta['preceding_vx'] = abs(follow_meta['preceding_vx'])

        return follow_meta
    
    def filter_follow_meta_by_initial_distance(follow_meta, max_distance):
        new_follow_meta = follow_meta.copy()
        for scenario in new_follow_meta.scenario_id.unique():
            df = new_follow_meta[new_follow_meta['scenario_id'] == scenario]
            distance = np.sqrt((df['ego_x'].iloc[0] - df['preceding_x'].iloc[0])**2 + (df['ego_y'].iloc[0] - df['preceding_y'].iloc[0])**2)
            if distance > max_distance:
                follow_meta = follow_meta[follow_meta['scenario_id'] != scenario]
        return follow_meta
    
    