import random
import matplotlib.pyplot as plt
from math import *
import numpy as np

class Robot:
  pos = 0

  predicted_pos = 0
  prediction_variance_2 = 10

  variance_estimate = None

  gps_variance = 5

  encoder_variance = 0.01

  k_constant = 0.8


  previous_variance = 0

  def __init__(self, pos):
    self.pos = pos
    self.time = 0
    self.velocity = 10
    # drift error as a percentage
    self.drift_error = (random.random() - 0.5) * 0.1 
    self.d_t = 0.1
    self.predicted_pos = self.get_gps_position()
    self.prediction_variance_2 = self.gps_variance

  def get_gps_position(self):
    return random.gauss(self.pos, self.gps_variance)
  
  def get_encoder_position(self):
    return random.gauss(self.pos, self.encoder_variance)

  def update(self):
    self.time += self.d_t
    if self.time < 5:
      self.pos = self.pos + self.velocity * self.d_t 
    # self.encoder_variance = self.encoder_variance + self.velocity ** 2

    # Compute the predicted position, this will be the mean of the previous probability distribution mulitiplied by the

    # self.predicted_pos = (self.encoder_variance * self.predicted_pos + self.prediction_variance_2 * self.get_encoder_position()) / (self.encoder_variance + self.prediction_variance_2)
    # self.prediction_variance_2 = self.encoder_variance * self.prediction_variance_2 / (self.encoder_variance + self.prediction_variance_2)

    # estimate_pos = ((self.encoder_variance * self.get_gps_position()) + self.gps_variance * self.get_encoder_position()) / ((self.encoder_variance + self.gps_variance))

    # estimate_variance = (self.encoder_variance * self.gps_variance) / (2 * (self.encoder_variance * self.k_constant + self.gps_variance * (1-self.k_constant)))

    position_estimate_raw, variance_estimate_raw = multiply_probability_distribution(self.get_gps_position(), self.gps_variance, self.get_encoder_position(), self.encoder_variance)
    if self.variance_estimate == None:
      self.variance_estimate = variance_estimate_raw

    print(position_estimate_raw, self.variance_estimate)

    self.predicted_pos, self.variance_estimate = multiply_probability_distribution(position_estimate_raw, variance_estimate_raw, self.predicted_pos, self.variance_estimate)

    # self.variance_estimate = max(self.variance_estimate, 0.1)
    
    if self.time < 5:
      self.variance_estimate += 2

    return [self.pos, self.predicted_pos]

def multiply_probability_distribution(mean1, var1, mean2, var2):
  new_mean = (var1 * mean2 + var2 * mean1) / (var1 + var2)
  new_var = var1 * var2 / (var1 + var2)
  return [new_mean, new_var]

def PI(variables):
  product = 1
  for var in variables:
    product *= var
  return product

def multiply_probability_distributions(means, vars):
  new_mean = sum([var * mean for var, mean in zip(vars, means)]) / sum(vars)
  new_var = PI(vars) / sum(vars)
  return [new_mean, new_var]

f = lambda x, mu, sigma: (exp(-0.5 * (x - mu) ** 2 / sigma ** 2) / (sigma * sqrt(2 * pi)))

mean1 = 30
var1 = 10
mean2 = 60
var2 = 10
mean3 = 90
var3 = 10

mean, var = multiply_probability_distributions([mean1, mean2, mean3], [var1, var2, var3])
# mean, var = multiply_probability_distribution(mean3, var3, mean2, var2)
# mean, var = multiply_probability_distribution(mean, var, mean1, var1)
# for i in range(100):
#   mean, var = multiply_probability_distribution(mean, var, mean2, var2)
# mean, var = multiply_probability_distributions([mean1, mean2], [var1, var2])
# mean, var = multiply_probability_distributions([mean, mean2], [var, var2])
# mean, var = multiply_probability_distributions([mean, mean2], [var, var2])
# for i in range(10):
#   mean, var = multiply_probability_distributions([mean, mean2], [var, var2])

print(mean, var)
x_data = np.arange(0, 100, 0.1)
y_data_1 = [f(x, mean1, var1) for x in x_data]
y_data_2 = [f(x, mean2, var2) for x in x_data]
y_data_3 = [f(x, mean3, var3) for x in x_data]
y_data = [f(x, mean, var) for x in x_data]

plt.plot(x_data, y_data_1, 'ro', label='1')
plt.plot(x_data, y_data_2, 'bo', label='2')
plt.plot(x_data, y_data_3, 'go', label='3')
plt.plot(x_data, y_data, 'yo', label='3')

plt.show()

raise

r = Robot(100)

reals = []
estimates = []
for i in range(100):
  pos, prediction = r.update()

  # plt.plot([pos, prediction], [f(pos, pos, r.gps_variance), f(prediction, pos, r.prediction_variance_2)], 'ro')
  # plt.show()

  reals.append(pos)
  estimates.append(prediction)

plt.plot(np.arange(len(reals)) , reals, 'ro', label='Real')
plt.plot(np.arange(len(estimates)) , estimates, 'bo', label='Estimate')
plt.legend()
plt.show()