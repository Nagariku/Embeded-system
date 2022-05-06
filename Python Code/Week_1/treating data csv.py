# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 15:27:04 2022

@author: admin

"""
import csv
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import numpy as np

def extract_data(csv_file):
    dataList =[]
    # opening the CSV file
    with open(csv_file, mode ='r')as file: 
        # reading the CSV file
        csvFile = csv.reader(file)
        
        # displaying the contents of the CSV file
        for lines in csvFile:
            dataList.append(lines)
            
    return dataList

def tick_to_rad(val):
    # 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
    val = val * 0.001533981
    return val

"""
# PART ONE: TREATING 'digested' values for linear movement, digested = preworked in excel, into a useful format
fileList = extract_data("linear digested.csv")

# The code below converts the saved time in the correct format, eg 500830078125 becomes 5.0083
for i in fileList:
    #print(i[3])
    num1 = str(i[3])
    num2 = ""
    count = 0
    while count < 5 and count <= len(num1)-1:
        num2 = num2 + num1[count]
        if count == 0 and num1[1] != ".":
            num2 = num2 + "."
        count += 1
    i[3] = num2 
    
# Convertes the list into a float list
floatList = []
for i in fileList:
    tempList = []
    for j in i:
        tempList.append(float(j))
    floatList.append(tempList)

xList = []
yList = []
refTick = (floatList [0][0] + floatList [0][1])/2 # the starting tick
for i in floatList:
    yVal = tick_to_rad(((i[0]+i[1])/2)-refTick) # take the avg of left and right tick, converts to radian
    yList.append(yVal)
    xList.append(i[3])
    
# Convert into m/s, when time is 0, the value is removed from the list
myCounter = 0
while  len(xList) - myCounter >0:
    if  yList[myCounter] == 0:
        yList.pop(myCounter)
        xList.pop(myCounter)
        myCounter = myCounter - 1
        
    yList[myCounter] = yList[myCounter]/xList[myCounter] * 0.066 # radius of wheel, rad/s to m/s
    myCounter += 1
    
xList = np.asarray(xList)
yList = np.asarray(yList)
B_spline_coeff = make_interp_spline(xList, yList)
X_Final = np.linspace(xList.min(), xList.max(), 25) #choose the resolution as 50
Y_Final = B_spline_coeff(X_Final)

plt.plot(X_Final, Y_Final)
plt.ylabel('Linear speed (m/s)') #set the label for y axis
plt.xlabel('Time (s)') #set the label for x-axis
plt.title("Linear mvt, ticks about time") #set the title of the graph
plt.grid()
plt.show() #display the graph
"""
# PART TWO: TREATING 'digested' values for angular movement
fileList = extract_data("angular digested.csv")

# The code below converts the saved time in the correct format, eg 500830078125 becomes 5.0083
for i in fileList:
    #print(i[3])
    num1 = str(i[3])
    num2 = ""
    count = 0
    while count < 5 and count <= len(num1)-1:
        num2 = num2 + num1[count]
        if count == 0 and num1[1] != ".":
            num2 = num2 + "."
        count += 1
    i[3] = num2 
    
# Convertes the list into a float list
floatList = []
for i in fileList:
    tempList = []
    for j in i:
        tempList.append(float(j))
    floatList.append(tempList)

xList = []
yList = []
refTick = (floatList [0][0] + floatList [0][1])/2 # the starting tick
for i in floatList:
    yVal = tick_to_rad(((i[0]+i[1])/2)-refTick) # take the avg of left and right tick, converts to radian
    yList.append(yVal)
    xList.append(i[3])
    
# Convert into m/s, when time is 0, the value is removed from the list
myCounter = 0
while  len(xList) - myCounter >0:
    if  yList[myCounter] == 0:
        yList.pop(myCounter)
        xList.pop(myCounter)
        myCounter = myCounter - 1
        
    yList[myCounter] = yList[myCounter]/xList[myCounter] * 0.066 # radius of wheel, rad/s to m/s
    myCounter += 1
    
xList = np.asarray(xList)
yList = np.asarray(yList)
B_spline_coeff = make_interp_spline(xList, yList)
X_Final = np.linspace(xList.min(), xList.max(), 250) #choose the resolution as 50
Y_Final = B_spline_coeff(X_Final)

plt.plot(X_Final, Y_Final)
plt.ylabel('Angular speed (m/s)') #set the label for y axis
plt.xlabel('Time (s)') #set the label for x-axis
plt.title("Angular mvt, ticks about time") #set the title of the graph
plt.grid()
plt.show() #display the graph

