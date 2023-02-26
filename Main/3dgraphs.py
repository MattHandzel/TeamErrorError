import pandas as pd
import numpy as np
import datetime
import matplotlib.pyplot as plt

import os

all_data = [pd.read_csv("../" + f) for f in os.listdir("../") if "AllVexData" in f]

all_data = [all_data[-1], all_data[1], all_data[4], all_data[3], all_data[0], all_data[2]]



all_data = [z[z["Event Region"] == "Illinois"] for z in all_data]
# a = allData

# Filter out where the data does not have a driver score for the skills
all_data = [z[pd.notna(z["Highest Driver Score Timestamp"])] for z in all_data]

dates = [[z.split(" ")[0] for z in x["Highest Driver Score Timestamp"][pd.notna(x["Highest Driver Score Timestamp"])]] for x in all_data]
dates = [[[int(y) for y in z.split("-")] for z in x] for x in dates]

timedates = []

for i, date_array in enumerate(dates):
  timedates.append([])
  for date in date_array:
    timedates[i].append((datetime.datetime(date[0], date[1], date[2], 0, 0).timestamp() // (3600 * 24 * 7)) * (3600 * 24 * 7))
for i, data in enumerate(all_data):
  data["Datetime"] = timedates[i]

all_teams = {}

for data in all_data:
  for team in data["Team Number"]:
    if team not in all_teams:
      all_teams[team] = []

for data in all_data:
  for i, team in enumerate(data["Team Number"]):
    all_teams[team].append([list(data["Datetime"])[i], list(data["Score"])[i]])

scores = []
times = []
start_times = []
for team in all_teams:
  team_scores = [z[1] for z in all_teams[team]]
  team_times = np.array([z[0] for z in all_teams[team]])

  team_scores_unique = np.unique(team_scores)

  team_times = team_times[np.array([team_scores.index(t) for t in team_scores_unique])]
  
  team_scores = team_scores_unique
  if len(team_scores) > 1:
    for i in range(len(team_scores) - 1):
      scores.append(team_scores[i + 1] - team_scores[i])
      times.append(team_times[i + 1] - team_times[i])
      start_times.append(team_times[i])
  
# plt.plot(np.array(times) / (3600 * 24), scores, "o")

# Make a 3d graph where the x axis is the start_times array, y axis is times array, and z axis is scoares array
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter((-(np.array(start_times) - min(start_times)) / (3600 * 24)), np.array(times) / (3600 * 24), scores)


plt.show()