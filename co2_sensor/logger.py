import serial
import time
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('/dev/ttyACM1')
ser.flushInput()

plot_window = 20
y_var = np.array(np.zeros([plot_window]))

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(y_var)

t = 0
while True:
    try:
        
        ser_bytes = ser.readline()
        try:
            decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
            print(decoded_bytes)
        except:
            continue
        with open("test_data.csv","a") as f:
            writer = csv.writer(f,delimiter=",")
            writer.writerow([t,decoded_bytes])
        t+= 0.1
        time.sleep(0.1)
    except:
        print("Keyboard Interrupt")
        break