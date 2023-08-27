from enum import Enum
import statistics
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class Filter():
    
    @staticmethod
    def filter_dataframe(dataframe, *args):
        filtered_df = dataframe.copy()
        for i in range(0, len(args), 2):
            column_name = args[i]
            column_value = args[i + 1]
            print(f'Filtering by colName {column_name}, val {column_value}')
            if isinstance(column_value, tuple) or isinstance(column_value, list):
                min_value, max_value = column_value
                filtered_df = filtered_df[(filtered_df[column_name] >= min_value) & (filtered_df[column_name] <= max_value)]
            else:
                filtered_df = filtered_df[filtered_df[column_name] == column_value]
        print(f'Filtered dataframe from {len(dataframe)} to {len(filtered_df)} rows, ratio {len(filtered_df) / len(dataframe)}')
        return filtered_df

    # possible arg example. ('class', 'Car', 'lanechange', 0, )
    @staticmethod
    def _filter_actors(dataframe, *args):
        df = dataframe.copy()
        actors = df['id'].unique()
        nActors = len(actors)
        for i in range(0, len(args), 2):
            column_name = args[i]
            column_value = args[i + 1]

            if 'class' in column_name:
                filtered_actor_df = df[df[column_name] == column_value]
                filtered_actor = filtered_actor_df['id'].unique()

            if 'nLane' in column_name:
                grouped = df.groupby('id')
                filtered_actor = []
                for actor, group in grouped:
                    unique_lane_ids = group['laneId'].nunique()
                    if unique_lane_ids == column_value:
                        filtered_actor.append(actor)

            actors = np.intersect1d(actors, filtered_actor)

        print(f'total actors {nActors}, filtered actors {len(actors)}, ratio {len(actors) / nActors}')
        
        return actors

    
    
    

        

