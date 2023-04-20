import numpy as np
import matplotlib.pyplot as plt

def y_lin(x, xref):
    return (x-xref)**2

def y_circle(x, xref):
    radius_setp = 2
    return ((xref-x)**2-radius_setp**2)**2


x0 = 0
xref = 5

x = np.arange(0.0, 5.0, 0.01)


fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(x, y_lin(x, xref), 'r', label='Linear')
ax1.legend(loc='upper left')

ax2.plot(x, y_circle(x, xref+2), 'b', label='Circle')
ax2.legend(loc='upper left')

plt.show()