import serial
import re
import numpy as np
import math

numReadings = 10
readingsA0 = [0.0] * numReadings
readingsA1 = [0.0] * numReadings
readingsA2 = [0.0] * numReadings
readIndex = 0
total = 0.0
avgA0 = 0.0     #A0 => ancho main
avgA1 = 0.0     #A1 => ancho 1
avgA2 = 0.0     #A2 => ancho 2

ser = serial.Serial('COM9', 115200)
def rme(lst):
    lst.sort()
    lst = lst[1:-1]
    return lst

def find_intersection(A0, A1, A2):
    x1, y1, r1 = A0
    x2, y2, r2 = A1
    x3, y3, r3 = A2
    
    A = np.array([[2 * (x2 - x1), 2 * (y2 - y1)],
                  [2 * (x3 - x1), 2 * (y3 - y1)]])
    B = np.array([(r1**2 - r2**2 + x2**2 - x1**2 + y2**2 - y1**2),
                  (r1**2 - r3**2 + x3**2 - x1**2 + y3**2 - y1**2)])
    x, y = np.linalg.solve(A, B)
    return x, y

for i in range(numReadings):
    readingsA0[i] = 0.0

while True:
    A0 = float(ser.readline().decode().strip())
    A1 = float(ser.readline().decode().strip())
    A2 = float(ser.readline().decode().strip())
    
    readingsA0[readIndex] = A0    #data
    readingsA1[readIndex] = A1
    readingsA2[readIndex] = A2
    
    srtA0 = rme(readingsA0)
    srtA1 = rme(readingsA1)
    srtA2 = rme(readingsA2)
    # print(srtA0)
    # print(srtA1)
    # print(srtA2)
    
    sumA0 = sum(srtA0)
    sumA1 = sum(srtA1)
    sumA2 = sum(srtA2)
    
    # numA0 = sum(srtA0)
    # numA1 = sum(srtA1)
    # numA2 = sum(srtA2)
    # print(numA0)
    # print(numA1)
    # print(numA2)
    
    
    readIndex = (readIndex + 1) % numReadings
    print('readIndex',readIndex)
    
    avgA0 = sumA0 / (numReadings - 2)
    avgA1 = sumA1 / (numReadings - 2)
    avgA2 = sumA2 / (numReadings - 2)
    # print('avgA0 ',avgA0)
    # print('avgA1 ',avgA1)
    # print('avgA2 ',avgA2)

    
    avg_distanceA0 = (0, 0, avgA0)
    avg_distanceA1 = (2.6, 0, avgA1)
    avg_distanceA2 = (0, 3.3, avgA2)
    x, y = find_intersection(avg_distanceA0, avg_distanceA1, avg_distanceA2)
    print(f"Intersection Point: ({x:.4f}, {y:.4f})".format(x, y))
    print('\n')