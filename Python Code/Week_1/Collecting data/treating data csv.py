# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 15:27:04 2022

@author: admin

"""
import csv

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

fileList = extract_data("Saved data linear air full data.csv")

print(fileList)