import matplotlib.pyplot as plt
import numpy as np

def circle_sp(x, x_d, radius):
    return (25/12544)*(((x_d-x)**2)-radius**2)**2

def circle_sp_derivated(x, x_d, radius):
    return (25/12544)*(-4*((x_d-x)**2-1)*(x_d-x))

def quad_sp(x, x_d):
    return (4/9)*((x_d-x)**2)

def quad_sp_derivated(x, x_d):
    return (4/9)*(-2*(x_d-x))


def circle_sp_better(x,x_d, radius):
    return (100/((np.sqrt(x_d**2)-radius))**2)*(np.sqrt((x_d-x)**2)-radius)**2

def circle_sp_better_derivated(x,x_d, radius):
    return (100/((np.sqrt(x_d**2)-radius))**2)*((2*(abs(x-x_d)-radius)*(x-x_d))/(abs(x-x_d)))

x1 = np.linspace(13, 17, 100)
x2 = np.linspace(13, 17, 100)

#plt.plot(x1, circle_sp(x1, 15, 1), label="Implemented Circular setpoint")
#plt.plot(x1, circle_sp_better(x1, 15, 1), label="Optimal Circular setpoint")
#plt.plot(x2, quad_sp(x2, 15), label="Implemented Squared error")

plt.plot(x1, circle_sp_derivated(x1, 15, 1), label="Implemented Circular setpoint")
plt.plot(x1, circle_sp_better_derivated(x1, 15, 1), label="Optimal Circular setpoint")
plt.plot(x2, quad_sp_derivated(x2, 15), label="Implemented Squared error")

plt.ylabel("Derivative of Cost")
plt.xlabel("X [m]")
plt.title("Comparison of Positioning Approaches in Cost Function")
plt.grid()
plt.legend()
plt.savefig("cost_function_comparison_derivative_disc.png", dpi=300)
plt.show()
