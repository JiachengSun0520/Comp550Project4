from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('path.txt')
#fig = plt.figure()
#ax = fig.gca(projection = '3d')
#ax.plot(data[:,0],data[:,1])

plt.plot(data[:,0], data[:,1])
plt.show()
