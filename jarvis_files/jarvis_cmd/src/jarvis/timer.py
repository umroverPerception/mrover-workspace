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
import os

def timerFunc():
    functions = []
    times = []
    segs = []
    count = 0

    #open file that will have data about program run
    #print("Timing test")
    fileDir = os.path.dirname(os.path.realpath('__file__'))
    print(fileDir)
    fileName = os.path.join(fileDir, 'jetson/percep/timerInput.txt')
    file = open(fileName)
    lines = file.readlines()

    # parse data
    for line in lines:
        segs.append(line.split())

    for seg in segs:
        functions.append(seg[0])
        times.append(float(seg[1]))

    for f, t in zip(functions,times):
        print(f, t)
    
    print('\n')
    # find mean, median, standard deviation of times
    totalTime = sum(times)
    medianTime = stat.median(times)
    meanTime = stat.mean(times)
    stdevTime = stat.stdev(times)

    print("Total time to run: %.2f s " % totalTime)
    print("Mean: %.2f s" % round(meanTime,2))
    print("Median: %.2f s" % round(medianTime,2))
    print("StDev: %.2f s" % round(stdevTime,2))

    # writing information to the file
    fileName_out = os.path.join(fileDir, 'jetson/percep/timerOutput.txt')
    print(fileName_out)
    out = open(fileName_out, "w")
    out.write("Total time to run: %.2f s \n" % totalTime) 
    out.write("Mean: %.2f s \n" % round(meanTime,2))
    out.write("Median: %.2f s \n" % round(medianTime,2))
    out.write("StDev: %.2f s \n" % round(stdevTime,2))

    # function_name time

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