import string
import random
import cv2 as cv
from cv2 import THRESH_BINARY
import numpy as np 
from math import ceil,radians
import imutils
import yaml

class MapGenerator:
  def __init__(self,worldSize,resolution,name,path):
    self.resolution = resolution
    self.size = [ceil(worldSize[0]/resolution),ceil(worldSize[1]/resolution)]
    self.name = name
    self.path = path
    self.maxVal = 255
    self.img = np.zeros((self.size[1],self.size[0],1), dtype=np.uint8)

  def mapToImgFrame(self,pos):
    return [ceil(self.size[0]/2+(pos[0]/self.resolution)),ceil(self.size[1]/2+(pos[1]/self.resolution))]
  
  def generateMap(self,objects):
    for obj in objects:
      if obj['type']=='box':
        self.drawPolygon(obj['pos'][:2],obj['rot'][2],obj['scale'][:2])
      if obj['type']=='cylinder':
        self.drawCircle(obj['pos'][:2],obj['scale'][0])
      if obj['type']=='sphere':
        self.drawCircle(obj['pos'][:2],obj['scale'][0])
    self.img = np.resize(self.img,self.img.shape[:2])
    #self.img = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
    _,self.img = cv.threshold(self.img,1,255,cv.THRESH_BINARY_INV)
    self.save()

  def drawPolygon(self,pos,rot,scale):
    x = ceil(scale[1]/self.resolution)
    y = ceil(scale[0]/self.resolution)
    poly = np.ones((x,y,1), dtype=np.uint8)*2
    rotated = imutils.rotate_bound(poly, rot)
    rotated = np.expand_dims(rotated, axis=2)
    objPos = self.mapToImgFrame(pos)
    #self.overlay_image_alpha(self.img,rotated,*objPos)
    self.addObjToImg(rotated,objPos)

  def drawCircle(self,pos,radius):
    objPos = self.mapToImgFrame(pos)
    self.img = cv.circle(self.img, (objPos[1],objPos[0]), ceil(radius/self.resolution), (2), -1)

  def addObjToImg(self,obj,objPos):
    #check if the left size fit
    yImgStart = max(0,objPos[1]-ceil(obj.shape[0]/2))
    yImgEnd = min(self.img.shape[0],objPos[1]+(obj.shape[0]-ceil(obj.shape[0]/2)))
    xImgStart = max(0,objPos[0]-ceil(obj.shape[1]/2) )
    xImgEnd = min(self.img.shape[1],objPos[0]+(obj.shape[1]-ceil(obj.shape[1]/2)))

    yRotStart = max(0,-(objPos[1]-ceil(obj.shape[0]/2)))
    yRotEnd = min(obj.shape[0],((obj.shape[0]-ceil(obj.shape[0]/2))  + (self.img.shape[0]-objPos[1])))
    xRotStart = max(0,-(objPos[0]-ceil(obj.shape[1]/2) ))
    xRotEnd = min(obj.shape[1],(obj.shape[1]-ceil(obj.shape[1]/2))  + (self.img.shape[1]-objPos[0]))
    
    self.img[yImgStart:yImgEnd,xImgStart:xImgEnd]+=obj[yRotStart:yRotEnd,xRotStart:xRotEnd]

  def save(self):
    cv.imwrite(f"{self.path+self.name}.pgm",self.img)
    ymlString= f"""
    image: ./{self.name}.pgm
    resolution: {self.resolution}
    origin: [{-self.resolution*self.size[0]/2}, {-self.resolution*self.size[1]/2}, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
    """
    ymlFile = {"image": f"./{self.name}.pgm",
    "resolution": self.resolution,
    "origin": [-self.size[0]/2, -self.size[1]/2, 0.000000],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196}
    with open(f"{self.path+self.name}.yaml", 'w') as f:
      #data = yaml.dump(ymlFile, f)
      f.write(ymlString)
    print( f"files {self.name}.pgm, {self.name}.yaml, has been created at {self.path} ")
