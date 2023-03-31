# Github Copilot, write me a function that checks to see if two line segments intersect

def check_intersection(line1, line2):
    # line1 and line2 are tuples of 4 points (x1, y1, x2, y2)
    # returns True if they intersect, False otherwise
    
    # get the slope and y-intercept of each line
    m1 = (line1[3] - line1[1]) / (line1[2] - line1[0])
    m2 = (line2[3] - line2[1]) / (line2[2] - line2[0])
    b1 = line1[1] - m1 * line1[0]
    b2 = line2[1] - m2 * line2[0]
    
    # if the slopes are equal, they are parallel and don't intersect
    if m1 == m2:
        return False
    
    # find the point of intersection
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    
    # check if the point of intersection is within the line segments
    if x > max(line1[0], line1[2]) or x < min(line1[0], line1[2]):
        return False
    if x > max(line2[0], line2[2]) or x < min(line2[0], line2[2]):
        return False
    if y > max(line1[1], line1[3]) or y < min(line1[1], line1[3]):
        return False
    if y > max(line2[1], line2[3]) or y < min(line2[1], line2[3]):
        return False
    
    return True

# print(check_intersection([0,0, 1,1], [2.1,0, 0,2.1]))
line1 = [0.008572135, 0.008668765, 0.6713714, 16.75096] 

line2 = [-7.0, 24.99864, 7.0, 25.00136]
import matplotlib.pyplot as plt
plt.plot([line1[0], line1[2]], [line1[1], line1[3]], 'r')
plt.plot([line2[0], line2[2]], [line2[1], line2[3]], 'b')
plt.show()
