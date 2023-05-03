import numpy as np
import pandas as pd

df1 = pd.read_csv("data1.csv")
df2 = pd.read_csv("data2.csv")

#print(df1['yaw_ref'], df1['psi'])
#
#print(df2['yaw_ref'][780:800])

dfa = df2[780:800]
dfb = df1[780:800]

v1 = 0

for i in range(780, 800):
    v = [dfb['x'][i]-dfa['x'][i], dfb['y'][i]-dfa['y'][i]]
    print(np.arctan2(v[1], v[0]))