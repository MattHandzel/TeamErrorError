state = [{'x_pos': -25.79604, 'theta': 1.248835, 'y_pos': -0.2300973, 'intake_speed': 0, 'launch_expansion': False, 'flywheel_speed': 0}, {'x_pos': -8.947994, 'theta': 0.03459766, 'y_pos': -30.16593, 'intake_speed': 0, 'launch_expansion': False, 'flywheel_speed': 0}, {'x_pos': 26.28274, 'theta': 357.6559, 'y_pos': -29.86531, 'intake_speed': 0, 'launch_expansion': False, 'flywheel_speed': 0}, {'x_pos': 25.80928, 'theta': 94.28954, 'y_pos': -29.67564, 'intake_speed': 0, 'launch_expansion': False, 'flywheel_speed': 0}, {'x_pos': -0.4682667, 'theta': 10.44668, 'y_pos': -1.178292, 'intake_speed': 0, 'launch_expansion': False, 'flywheel_speed': 0}]

def print_state_nicely(state):
  # nicely format the state dictionary

  string = "["
  for s in state:
    string += "\n  {"
    for key in s:
      string += "\n    " + "\"" + key + "\"" + ": " + str(s[key]) + ","
    string += "\n  },"

  string += "\n]"
  return string

print_state_nicely(state)

