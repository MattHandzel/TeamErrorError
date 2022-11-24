import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = []
with open("./data2.txt", "r") as f:
  raw = f.read()
  raw = raw.split("\n")
  for row in raw:
    
    if len(row.split(" ")) == 9 and not '' in row.split(" "):
      if max([str.count(s, '.') for s in row.split(" ")]) <= 1:
        data.append(row.split(" "))

df = pd.DataFrame(data)
df.pop(0)
df.pop(3)
df.pop(6)
df.pop(7)
df.pop(8)


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(np.array(df[1], dtype=float), np.arange(len(df[1])), np.array(df[2], dtype=float), cmap='Reds')
ax.scatter3D(np.array(df[4], dtype=float), np.arange(len(df[1])), np.array(df[2], dtype=float), cmap="Greens")

plt.show()