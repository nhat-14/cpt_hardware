import serial

ARDUINO_PORT = "/dev/ttyACM0"   # serial port of Arduino
BAUD_RATE = 9600                # arduino uno runs at 9600 baud
FILE_NAME = "analog-data.csv"   # name of the CSV file generated
N_SAMPLE = 100                   # how many samples to collect

ser = serial.Serial(ARDUINO_PORT, BAUD_RATE)
print("Connected to Arduino port:" + ARDUINO_PORT)

line = 0 #start at 0 because our header is 0 (not real data)
while line <= N_SAMPLE:
    if line==0:
        print("Printing Column Headers")
    else:
        print("Line " + str(line) + ": writing...")
    get_data = str(ser.readline())
    data = get_data[0:][:-2]

    file = open(FILE_NAME, "a")
    file.write(data + "\n") #write data with a newline
    line += 1
    
file.close()
print("Data collection complete!")