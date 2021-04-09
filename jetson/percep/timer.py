''' 
GOAL: 
- create a Python script that can be run by jarvis for timing our code

IDEA: 
- script builds and executes perception on the command line 
- write runtime for each function to file 
- after running, find median, mean, and std. dev execution time of each function + 
as well as these statistics for the fps of the code. 

RESULT: 
- new function to jarvis called "test". 
- be able to run ./jarvis test jetson/percep timing, which will execute the python script 
and should then seamlessly provide timing output for our code.
'''

import statistics as stat
import numpy as np

#open file that will have data about program run
print("Timing test")
file = open("testOutput.txt")
lines = file.readlines()

# parse data
for line in lines:
    seg = line.split() 
for i in range(0, len(seg), 2):
    fileNames.append(seg[i])
for i in range(1, len(seg), 2):
    times.append(seg[i])

# find mean, median, standard deviation of times
totalTime = sum(times)
medianTime = stat.median(times)
meanTime = stat.mean(times)
stdevTime = stat.stdev(times)

print("Total time to run: %d s" % totalTime)
print("Mean: %d s" % meanTime)
print("Median: %d s" % medianTime)
print("StDev: %d s" % stdevTime)

# function_name time
# writing information to the file

# parse output file => ASHWIN'S SAMPLE CODE 
'''print("integ check")
f = open("noXserverPCLFPS1minMotion2.txt")
lines = f.readlines()
​
for line in lines:
	#print(line)
	seg = line.replace(" ", "")
	seg = line.split(":")
	#print(seg[0])
	if(seg[0] == "FPS Iteration"):
		print(seg[1])
#print("hi")
'''
# main program
# def main(): 
    # time code 
    # write times to output file 
    # parse output file to median, mean, std 