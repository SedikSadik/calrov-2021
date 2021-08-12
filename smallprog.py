import os
  
# Directory
directory = "GeeksForGeeks"
  
# Parent Directory path
parent_dir = "/home/violetcheese/Documents/CALROV/"
  
# Path
path = os.path.join(parent_dir, directory)
print(path)
print(parent_dir+directory)
# Create the directory
# 'GeeksForGeeks' in
# '/home / User / Documents'
os.mkdir(path)