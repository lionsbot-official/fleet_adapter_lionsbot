class Coord:
   def __init__(self, 
              x: float, 
              y: float, 
              orientation_radians: float = None):
      self.x = x
      self.y = y
      self.orientation_radians = orientation_radians
   
   def __str__(self) -> str:
      if self.orientation_radians is None:
         return f'({self.x}, {self.y})'
      else:
         return f'({self.x}, {self.y}, {self.orientation_radians})'

class RmfCoord(Coord):
   pass
   
class LionsbotCoord(Coord):
   pass

