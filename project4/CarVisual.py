import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('CarRRT.txt')
#fig = plt.figure()
#ax = fig.gca(projection = '3d')
#ax.plot(data[:,0],data[:,1])

plt.plot(data[:,0], data[:,1])
rec1 = plt.Rectangle((-10,-4), 8, 8, fc='r')
rec2 = plt.Rectangle((1,-4), 8, 8, fc = 'r')
rec3 = plt.Rectangle((-10,6), 20, 2, fc = 'r')
rec4 = plt.Rectangle((-10,-8), 20, 2, fc = 'r')
rec5 = plt.Rectangle((-10,-8), 2, 15, fc = 'r')
rec6 = plt.Rectangle((8.5,-8), 2, 15, fc = 'r')

plt.gca().add_patch(rec1)
plt.gca().add_patch(rec2)
plt.gca().add_patch(rec3)
plt.gca().add_patch(rec4)
plt.gca().add_patch(rec5)
plt.gca().add_patch(rec6)
plt.show()
