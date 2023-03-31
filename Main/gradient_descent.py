import matplotlib.pyplot as plt
import numpy as np

class Vector:
    '''
    Vector class I wrote because basic python lists are lame, this is as slow as normal python, should be ideally replaced with numpy arrays
    '''
    def __init__(self, data):
        self.data = data

    def __add__(self, other):
        assert len(other) == len(self.data)
        return Vector([other[i] + self.data[i] for i in range(len(self.data))])

    def __sub__(self, other):
        assert len(other) == len(self.data)
        return Vector([self.data[i] - other[i] for i in range(len(self.data))])
  
    def __mul__(self, other):
        return Vector([x * other for x in self.data])
  
    def __rmul__(self, other):
        return Vector([x * other for x in self.data])
  
    def __rdev__(self, other):
        return Vector([other / x for x in self.data])
    
    def __truediv__(self, other):
        return Vector([x / other for x in self.data])

    def __getitem__(self, key):
        return self.data[key]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return "Vector:	" + repr(self.data)

class numpy:
    @staticmethod
    def abs(vector):
        return Vector([abs(x) for x in vector])

def magnitude(vector):
    return sum([x**2 for x in vector])**0.5



def compute_gradient(f, position):
  grads = []

  vectors = []

  for i, pos in enumerate(position):
    vectors.append(Vector([0] * i + [pos] + [0] * (len(position) - i - 1)))


  for vector in vectors:
    epsilon = 1e-2
    h = numpy.abs(vector.data) * epsilon * (1/max(numpy.abs(vector.data)))
    # print("h is", h)
    # print("vector plus h is", vector + h)
    # print("vector minus h is", vector - h)
    # print("f of vector + h: ", f(vector + h))
    # print("f of vector - h: ", f(vector - h))
    # print("dervative is", (f(vector + h) - f(vector - h)) * (1 / (2 * (epsilon))))

    grads.append((f(vector + h) - f(vector - h)) * (1 / (2 * (epsilon))))
  
  return grads

f = lambda x: -2*x[0]**2 - (x[1]-np.sqrt(2))**2 * np.sin(x[0]) - 1/10 * (x[2] ** 4)

guess_position = [(np.random.rand() - 0.5) * 10 ,(np.random.rand() - 0.5) * 10, (np.random.rand() - 0.5) * 10]
grad = compute_gradient(f, guess_position)
max_epsilon = 0.1
max_attempts = 100
epsilon = max_epsilon
attempts = 0
gradient_ascent_or_descend_constant = 1
positions = []
while attempts < max_attempts:
    noise = Vector([np.random.normal() for _ in range(len(guess_position))])
    delta_position = Vector(grad) * epsilon + noise * epsilon * (max_attempts - attempts) / max_attempts * 5
    # print("grad is", grad)
    # print("delta position is", delta_position)
    guess_position = Vector(guess_position) + delta_position * gradient_ascent_or_descend_constant
    # print("position is", guess_position)
    grad = compute_gradient(f, guess_position)
    positions.append(guess_position)
    attempts += 1
    epsilon = max_epsilon  
print("it took ", attempts, "attempts to find the minimum, final position is", guess_position, "with height of ", f(guess_position))
raise
f = np.vectorize(f)

x_axis = np.outer(np.linspace(-5, 5, 50), np.ones(50))
y_axis = x_axis.copy().T # transpose

# x_axis = np.linspace(-3, 3, 50)
# y_axis = np.linspace(-3, 3, 50)
# z_axis = np.linspace(-3, 3, 50)

f = lambda x, y: -2*x**2 - (y-np.sqrt(2))**2 * np.sin(y)

f = np.vectorize(f)

z_axis = f(x_axis, y_axis)

# 3d plot
fig = plt.figure()
ax = plt.axes(projection ='3d')
ax.plot_surface(x_axis, y_axis, z_axis, cmap='viridis')

# For every position in guess_position, plot the point in 3d plast
for position in positions:
    ax.scatter(position[0], position[1], f(position[0], position[1]), c='r')
    
plt.show()

