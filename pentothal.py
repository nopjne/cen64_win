#!/usr/bin/python

import os
import subprocess
import ctypes
import time
user32 = ctypes.windll.user32
screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
screensize = 11520,2160

print("size:" + str(screensize[0]) + " " + str(screensize[1]));

if not os.path.isfile("sodium64.z64"):
    print("Error: sodium64.z64 not found!")
    exit()

if not os.path.isdir("out"):
    os.mkdir("out")

maindir = ".//gm"
x = 0;
y = 0;
for filename in os.listdir(maindir):
    if (filename[-4:] == ".sfc" or filename[-4:] == ".smc" or filename[-4:] == ".SMC") and os.path.isfile(os.path.join(maindir, filename)):
        joinedfilepath = os.path.join(maindir, filename);
        print("Converting " + filename + "...")
        baseFile = open("sodium64.z64", "rb")
        inFile = open(joinedfilepath, "rb")
        if os.stat(joinedfilepath).st_size & 0x3FF == 0x200:
            inFile.seek(0x200) # Skip header
        outFile = open("out/" + filename[:-4] + ".z64", "wb")
        outFile.write(baseFile.read())
        outFile.write(inFile.read())
        time.sleep(0.3)
        parameters = "cen64.exe -windowoffset " + str(x) + " " + str(y)+ " -buttonreplay -compref -sram256k \"out\\" + filename[:-4] + ".ram\" pifdata.bin \"out\\" + filename[:-4] + ".z64\"";
        print("Starting CEN64 for " + filename + " :" + parameters);
        subprocess.Popen(parameters, shell=True)
        x = x + 640;
        if ((x + 640) > screensize[0]):
            x = 0;
            y += 480;
        if ((y + 480) > screensize[1]):
            break;



print("Done!")
