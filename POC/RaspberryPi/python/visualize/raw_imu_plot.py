import csv
import matplotlib.pyplot as plt
import numpy as np


#if __name__ == "__main__":
f = open("POC/RaspberryPi/visualize/data2.dat", 'r')
records = csv.reader(f)
datas = []
for record in records:
    datas.append(record)
datas.pop(0)
datas.pop(len(datas)-1)
a = np.array(datas, dtype=np.float)
plt.plot(a[:,0],'r-')
plt.plot(a[:,1],'g-')
#plt.plot(a[:,2],'b-')
#plt.axis([0, 2000, -100, 100])
plt.show()
plt.plot(a[:,3],'r-')
plt.plot(a[:,4],'g-')
plt.plot(a[:,5],'b-')
#plt.axis([0, 2000, -100, 100])
plt.show()
plt.plot(a[:,6], 'b-')
plt.show()

