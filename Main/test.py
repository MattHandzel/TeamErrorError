import math
arr = [2, -1]


def rotate_vector_2d(x, y, theta):
    '''
    Rotates a vector by theta degrees
    '''
    x_old = x
    x = x * math.cos(theta) - y * math.sin(theta)
    y = x_old * math.sin(theta) + y * math.cos(theta)

    return x, y


x,y = rotate_vector_2d(arr[0], arr[1], math.pi/2)
print(x,y)

