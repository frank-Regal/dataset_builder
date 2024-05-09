import os
import re
import csv
import yaml

# Define paths
directory = "/project/ws_dev/src/hri_cacti_xr/data/005_data/devices/mic/raw/" 
input_csv_path = "/project/ws_dev/src/hri_cacti_xr/data/005_data/annotations/labels.csv"
output_csv_path = "/project/ws_dev/src/hri_cacti_xr/data/005_data/annotations/labels_clean.csv"
yaml_path = "/project/ws_dev/src/hri_cacti_xr/dataset_builder/params/dicts.yaml"

# Load YAML data
with open(yaml_path, 'r') as file:
    data = yaml.safe_load(file)

# Extract the class names
class_names = list(data['class'].keys())

# Pattern to extract numbers after 'S'
pattern = re.compile(r"S(\d+)")

for i, class_name in enumerate(class_names):
    # Set Directory
    directory_to_search = directory + class_name + "/"
    print(directory_to_search)

    # Get the unique S numbers from the directory
    s_numbers = set()
    for filename in os.listdir(directory_to_search):
        match = pattern.search(filename)
        if match:
            s_numbers.add(match.group(1))

    print(len(s_numbers))

    # Filter lines from input CSV based on S numbers
    filtered_lines = []
    with open(input_csv_path, mode='r', newline='', encoding='utf-8') as input_csv:
        reader = csv.reader(input_csv, delimiter=' ')
        for row in reader:
            for s_number in s_numbers:
                if f"S{s_number}" in row[0]:
                    filtered_lines.append(row)
                    break

    # Write the filtered lines to the output CSV
    with open(output_csv_path, mode='a', newline='', encoding='utf-8') as output_csv:
        writer = csv.writer(output_csv, delimiter=' ')
        writer.writerows(filtered_lines)