import os

def getfilelist(file_dir):
    "merge text files"
    # Get a list of all files in the directory
    file_names = [f for f in os.listdir(file_dir) if os.path.isfile(os.path.join(file_dir, f))]

    return file_names