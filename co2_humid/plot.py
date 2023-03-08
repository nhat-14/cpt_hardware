import matplotlib.pyplot as plt
import datetime
  



import pandas as pd
import matplotlib.pyplot as plt

headers = ['value','time']
df = pd.read_csv('/home/nhat/Arduino/co2_humid/test_data.csv')




# df['timestamp'] = datetime.datetime.fromtimestamp(df['Date'])

df['time'] = df['time'].astype(float)


x = df['time']
y = df['value']

# plot
plt.plot(x,y)
plt.xlabel("Second (s)")
plt.ylabel("CO2 concentration (ppm)")
plt.axvspan(15.0, 17.0, ymin = 0, ymax = 16500,color="green", alpha=0.3)
# beautify the x-labels
# plt.gcf().autofmt_xdate()

plt.show()



# time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(1347517370))