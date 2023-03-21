import matplotlib.pyplot as plt
import numpy as np

###### DETTE KAN ENDRES PÅ FOR Å TESTE ######
ref = [5,5]
pos = [0,0]

avstand_sirkel = (pos[0]-(ref[0]+np.cos(ref[0])))+(pos[1]-(ref[1]+np.sin(ref[0])))

###### Plot greier ######
ax = plt.gca()
ax.cla()

# change default range so that new circles will work
ax.set_xlim((-10, 10))
ax.set_ylim((-10, 10))

circle2 = plt.Circle((ref[0], ref[0]), 1, color='b', fill=False)
ax.scatter(pos[0], pos[0], c='green')
ax.scatter(ref[0], ref[0], c='blue')
ax.add_patch(circle2)
plt.grid(True)
plt.legend(['SP', 'ROV'])

###### Disse er faktiske mål ########## 
avstand_SP = np.sqrt((ref[0]-pos[0])**2+(ref[1]-pos[1])**2)
fasktisk_avstand_sirkel = np.sqrt((pos[0]-ref[0])**2+(pos[1]-ref[1])**2) -  1

print("Avstand fra SP er: {}".format(avstand_SP))
print("Avstand fra sirkel er: {}".format(avstand_sirkel))
print("Riktig avstand fra sirkel er: {}".format(fasktisk_avstand_sirkel))

plt.show()