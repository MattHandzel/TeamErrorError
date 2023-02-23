import math
import time

# auto_orientate_dictionary = {
#   0 : controller_1.buttonX.pressing(),
#   -90 : controller_1.buttonY.pressing(),
#   90 : controller_1.buttonA.pressing(),
#   180 : controller_1.buttonB.pressing(),
# }

auto_orientate_dictionary = {
  0 : True,
  -90 : False,
  90 : False,
  180 : False,
}

auto_orientate_total_button_pressed = 1

total_theta = 0
for key in auto_orientate_dictionary:
  if auto_orientate_dictionary[key]:
    total_theta += key
print(total_theta / auto_orientate_total_button_pressed)  