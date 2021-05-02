import os
from sklearn.metrics import precision_recall_curve
from sklearn.metrics import PrecisionRecallDisplay
from sklearn.metrics import f1_score
from sklearn.metrics import auc
import numpy as np 
from matplotlib import pyplot

def accuracyFunc():
    print("hello from accuracyFunc")
    correct = []
    predicted = []
    segs = []

    #assuming input format: correct1 testresult1 
    fileDir = os.path.dirname(os.path.realpath('__file__'))
    print(fileDir)
    fileName = os.path.join(fileDir, 'jetson/percep/accuracyInput.txt')
    file = open(fileName)
    lines = file.readlines()

    # parse data
    for line in lines:
        segs.append(line.split())

    for seg in segs:
        correct.append(seg[0])
        predicted.append(float(seg[1]))
    '''
    true_pos = 0
    false_pos = 0
    false_neg = 0 
    true_neg = 0 
    
    for c,p in zip(correct,predicted):
        if c == p: true_pos += 1
        else if c == -1 and p != -1: false_pos +=1
        else if c != -1 and p == -1: false_neg +=1
        else if c != -1 and p != -1: true_neg +=1

    # precision-recall
    precision = true_pos / (true_pos + false_pos)
    recall = true_pos / (true_pos + false_neg)'''

    #plotting 
    precision, recall, threshold = precision_recall_curve(correct, predicted, pos_label=None, sample_weight=None)
    pyplot.plot(recall, precision)
    pyplot.show()
    pr_display = PrecisionRecallDisplay(precision=precision, recall=recall).plot()

    #f1 score calculation
    f1_score = 2*precision*recall/(precision+recall)
    print("FScore: %.2f s \n" % f1_score)
    
    #writing to output file 
    fileName_out = os.path.join(fileDir, 'jetson/percep/accuracyOutput.txt')
    print(fileName_out)
    out = open(fileName_out, "w")
    out.write("Precision: %.2f s \n" % precision)
    out.write("Recall: %.2f s \n" % recall)
    out.write("FScore: %.2f s \n" % f1_score)
