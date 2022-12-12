from utils import *
from peripherals import brain

class Button:

  call_back: function 

  def __init__(self, name = "", x = 0, y = 0, w = 0, h = 0, color = 0, call_back: function = function()):
    self.name = name
    self.x = x
    self.y = y
    self.w = w
    self.h = h
    self.color = color
    self.call_back = call_back

  def render(self):
    brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.color)
    brain.screen.print_at(self.name, x=self.x, y=self.y)
  
  def set_callback(self, function):
    self.call_back = function
  
  # If we do something like button() then it will run the callback function
  def __call__(self):
    self.call_back()

class Switch(Button):
  state = []
  def __init__(self, name = "", x = 0, y = 0, w = 0, h = 0, color = 0, call_back: function = function(), args = []):
    super().__init__(name, x, y, w, h, color, call_back)
    self.
  
  def __call__(self):
    self.call_back()

class GUI:
  '''
  What I want this class to do it to make a way for the drivers to interact with the brain screen and see some status
  things like if the motors are too hot, or if there is any self-diagnosed problem. I also want people to be able to select
  from the brain what team we're on.

  Brain screen dimensions: 480 x 240 pizels. Top left is (0,0)
  '''

  elements = []

  def __init__(self):
    pass

  def add_element(self, element):
    self.elements.append(element)

  def update(self):
    for element in self.elements:
      # If ANY element needs to re-render, then re-render everything
      if element.needs_to_render:
        break
    # If the for loops exists without a break statemenet
    else:
      self.render()
    
    # If the brain has been pressed ANYWHERE
    if brain.screen.pressed:
      # X and y positions of where the finger pressed
      x, y = brain.screen.x_position(), brain.screen.y_position()

      for element in self.elements:
        if abs(x - element.x) < element.w and (y - element.y) < element.h:
          element()
    
  
  def render(self):
    for element in self.elements:
      element.render()
    
    brain.screen.render()

