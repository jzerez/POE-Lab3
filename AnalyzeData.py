# AnalyzeData converts the distance and servo angles into
# x, y, z values using spherical coordinates. Plots and saves the data as a CSV file.

# Importing python libraries for plotting and analysis
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load txt file from SerialtoText and separate cols into separate arrays

data = np.loadtxt('onesensor_data.txt')
sensor1_values = data[:,0]
leftspeeds = data[:,1]
rightspeeds = data[:,2]


# Plotting 2d scatter of
x = np.linspace(0, 1, len(sensor1_values))
plt.fill(x, rightspeeds, 'b', x, leftspeeds, 'r', alpha=0.3)
plt.plot(x, rightspeeds, c='b', alpha=0.8)
plt.plot(x, leftspeeds, c='r', alpha=0.8)

# plt.plot([x[0], x[-1]], [30, 30], c='b', alpha=0.8)
# plt.plot([x[0], x[-1]], [0, 0], c='b', alpha=0.8)
#
# plt.plot(sensor1_values)
# plt.plot(15 * leftspeeds)
# plt.plot(15 * rightspeeds)

plt.legend(['Sensor Values','Left Motor Speeds','Right Motor Speeds'])
plt.grid()
# plt.scatter(sensor1_values, sensor2_values)
plt.show()
