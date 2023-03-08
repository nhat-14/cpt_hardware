import matplotlib.pyplot as plt
import datetime
  



import pandas as pd
import matplotlib.pyplot as plt

headers = ['Value','Time']
df = pd.read_csv('/home/nhat/Arduino/test/test_data.csv')




# df['timestamp'] = datetime.datetime.fromtimestamp(df['Date'])

df['Time'] = df['Time'].astype(float)


x = df['Time']
y = df['Value']

# plot
plt.plot(x,y)
plt.xlabel("Second (s)")
plt.ylabel("Sensor response (V)")
plt.axvspan(5.0, 6.5, ymin = 0, ymax = 5,color="green", alpha=0.3)
# beautify the x-labels
# plt.gcf().autofmt_xdate()

plt.show()



# time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(1347517370))