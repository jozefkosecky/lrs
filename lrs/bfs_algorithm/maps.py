from PIL import Image
import os
import re

class Maps:
    def __init__(self):
        self.directory_path = "/home/jozef/ros2_ws/src/lrs/maps"
        self.maps = self._load_maps(self.directory_path)

    def _load_maps(self, directory_path):
        map_collection = {}
        files = os.listdir(directory_path)

        # Filter only the .pgm files
        pgm_files = [file for file in files if file.endswith('.pgm')]

        for pgm_file in pgm_files:
            full_path = os.path.join(directory_path, pgm_file)
            
            # Remove the .pgm extension to get the key
            name = os.path.splitext(pgm_file)[0]
            key = str(int(re.findall(r'\d+', name)[0]))
            map_collection[key] = self._load_pgm_file(full_path)
        
        return map_collection
    
    def _load_pgm_file(self, file_path):
        with open(file_path, 'rb') as f:
            byte_data = f.read()
            data = byte_data.decode("utf-8")
            
        return data

# def load_pgm_file(file_path):
#     with open(file_path, 'rb') as f:
#         content = f.read()
#     return content

# def load_maps(directory_path):
#     map_collection = {}
#     files = os.listdir(directory_path)

#     # Filter only the .pgm files
#     pgm_files = [file for file in files if file.endswith('.pgm')]

#     for pgm_file in pgm_files:
#         full_path = os.path.join(directory_path, pgm_file)
        
#         # Remove the .pgm extension to get the key
#         name = os.path.splitext(pgm_file)[0]
#         key = str(int(re.findall(r'\d+', name)[0]))
#         map_collection[key] = load_pgm_file(full_path)
    
#     return map_collection

# def main():
#     directory_path = "/home/jozef/ros2_ws/src/lrs/maps"
#     maps = load_maps(directory_path)
#     map = maps["25"]
#     print(f"Loaded {len(maps)} PGM maps.")
    
# main()