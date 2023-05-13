import os
files = os.listdir()

new_files = []
for i in range(len(files)):
    new_files.append(files[i].replace("ideal", "default"))

for i in range(len(files)):
    try:
        os.rename(files[i], new_files[i])
    except FileNotFoundError:
        print(f"{files[i]} does not exist.")
print(new_files)