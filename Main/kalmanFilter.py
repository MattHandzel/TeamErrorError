import random
import matplotlib.pyplot as plt
from math import *
import numpy as np

class Robot:
  pos = 0

  predicted_pos = 0
  prediction_variance_2 = 10

  gps_sigma_2 = 5

  encoder_sigma_2 = 0.1

  k_constant = 0.8

  velocity = 1
  d_t = 0.01

  previous_sigma_2 = 0



  def __init__(self, pos):
    self.pos = pos

    # drift error as a percentage
    self.drift_error = (random.random() - 0.5) * 0.1 

    self.predicted_pos = self.get_gps_position()
    self.prediction_variance_2 = self.gps_sigma_2

  def get_gps_position(self):
    return random.gauss(self.pos, self.gps_sigma_2)
  
  def get_encoder_position(self):
    return self.velocity * self.d_t + random.gauss(self.predicted_pos, self.encoder_sigma_2)

  def update(self):
    self.pos = self.pos + self.velocity * self.d_t

    # Compute the predicted position, this will be the mean of the previous probability distribution mulitiplied by the

    # self.predicted_pos = (self.encoder_sigma_2 * self.predicted_pos + self.prediction_variance_2 * self.get_encoder_position()) / (self.encoder_sigma_2 + self.prediction_variance_2)
    # self.prediction_variance_2 = self.encoder_sigma_2 * self.prediction_variance_2 / (self.encoder_sigma_2 + self.prediction_variance_2)

    # estimate_pos = ((self.encoder_sigma_2 * self.get_gps_position()) + self.gps_sigma_2 * self.get_encoder_position()) / ((self.encoder_sigma_2 + self.gps_sigma_2))

    self.predicted_pos = ((self.encoder_sigma_2 * self.get_gps_position()) + (self.gps_sigma_2 * self.get_encoder_position())) /  (self.encoder_sigma_2 + self.gps_sigma_2)

    # estimate_variance = (self.encoder_sigma_2 * self.gps_sigma_2) / (2 * (self.encoder_sigma_2 * self.k_constant + self.gps_sigma_2 * (1-self.k_constant)))

    return [self.pos, self.predicted_pos]


f = lambda x, mu, sigma: (exp(-0.5 * (x - mu) ** 2 / sigma ** 2) / (sigma * sqrt(2 * pi)))
r = Robot(100)

r.velocity = 0



reals = []
estimates = []
for i in range(1000):
  pos, prediction = r.update()

  # plt.plot([pos, prediction], [f(pos, pos, r.gps_sigma_2), f(prediction, pos, r.prediction_variance_2)], 'ro')
  # plt.show()

  reals.append(pos)
  estimates.append(prediction)

plt.plot(np.arange(len(reals)) , reals, 'ro', label='Real')
plt.plot(np.arange(len(estimates)) , estimates, 'bo', label='Estimate')
plt.legend()
plt.show()