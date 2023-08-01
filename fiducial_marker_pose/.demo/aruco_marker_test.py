import numpy as np
import yaml
import sys
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# Function for processing data from the file
def get_values_from_txt(input_file, marker_id):
    with open(input_file, "r") as file:
        data = file.read()
    # Split data by "---" to obtain individual records
    entries = data.split("---")
    res = []
    for entry in entries:
        # Skip empty records
        if entry.strip() == "":
            continue
        # Load data from YAML record
        parsed_data = yaml.safe_load(entry)
        # Extract data and write to file
        m_id = parsed_data["markers"][0]["id"]
        pose_position_z = parsed_data["markers"][0]["pose"]["position"]["z"]
        if m_id == marker_id:
            res.append(pose_position_z)
    return res

def main():
    if len(sys.argv) == 1:
        path_to_config = 'aruco_marker_test.yaml'
    else:
        option = sys.argv[1]
        path = sys.argv[2]
        if option == '--config-dir':
            path_to_config = path
        else:
            print(f'Unknown option: {option}')
            return

    # Load data from YAML file
    with open(path_to_config, "r") as file:
        config = yaml.safe_load(file)

    # Get values from YAML file
    input_files = [entry["path"] for entry in config["input_files"]]
    marker_sizes = [entry["marker_size"] for entry in config["input_files"]]
    test_name = config["test_name"]
    marker_id = config["marker_id"]

    dfs = []
    for input_file, marker_size in zip(input_files, marker_sizes):
        values = get_values_from_txt(input_file, marker_id)
        df = pd.DataFrame({'distance': values, 'marker': np.repeat([marker_size], len(values))})
        dfs.append(df)

    # Combine all DataFrame into one
    df_res = pd.concat(dfs)

    # Create the plot
    sns.boxplot(x='marker', y='distance', data=df_res, showfliers=False)
    plt.title(test_name)
    plt.ylabel('Расстояние, м')
    plt.xlabel('Размер маркера, см')
    plt.show()

if __name__ == '__main__':
    main()
