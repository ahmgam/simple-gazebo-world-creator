import string
import random
import cv2 as cv
from cv2 import THRESH_BINARY
import numpy as np 
from math import ceil,radians
import imutils
import yaml
import map_creator
class GazeboWorldCreator:
    def __init__(self,name,path,size=[100,100],resolution=0.05):
        self.file = ""
        self.path = path
        self.counter = 0
        self.size = size
        self.name = name
        self.objects = []
        self.generator = MapGenerator(size,resolution,name,path)

    def _beginFile(self):        
        self.file += f"""
        <sdf version=\'1.4\'>
        <world name=\'{self.name}\'>
        <plugin name='ros_interface_plugin' filename='librotors_gazebo_ros_interface_plugin.so' />
            """
        
    def _defineConfig(self):
      self.file+="""
      <gravity>0 0 -9.8</gravity>
            <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
            <atmosphere type='adiabatic'/>
            <physics type='ode'>
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1</real_time_factor>
              <real_time_update_rate>1000</real_time_update_rate>
            </physics>
            <scene>
              <ambient>0.4 0.4 0.4 1</ambient>
              <background>0.7 0.7 0.7 1</background>
              <shadows>1</shadows>
            </scene>
            <wind/>
            <spherical_coordinates>
              <surface_model>EARTH_WGS84</surface_model>
              <latitude_deg>0</latitude_deg>
              <longitude_deg>0</longitude_deg>
              <elevation>0</elevation>
              <heading_deg>0</heading_deg>
            </spherical_coordinates>
      """
    def _insertSimConfig(self):
      self.file+="""
              <sim_time>121 490000000</sim_time>
              <real_time>121 754087262</real_time>
              <wall_time>1660613319 669022149</wall_time>
              <iterations>121490</iterations>
      """
    def _defineLights(self):
      self.file +=f"""
              <light name='sun' type='directional'>
              <cast_shadows>1</cast_shadows>
              <pose>0 0 10 0 -0 0</pose>
              <diffuse>0.8 0.8 0.8 1</diffuse>
              <specular>0.2 0.2 0.2 1</specular>
              <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
              <direction>-0.5 0.1 -0.9</direction>
              <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
              </spot>
            </light>
      """

    def _insertLights(self):
      self.file +=f"""
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
      """

    def _defineGroundPlane(self):
      self.file +=f"""
      <model name='ground_plane'>
              <static>1</static>
              <link name='link'>
                <collision name='collision'>
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>{self.size[0]} {self.size[1]}</size>
                    </plane>
                  </geometry>
                  <surface>
                    <contact>
                      <collide_bitmask>65535</collide_bitmask>
                      <ode/>
                    </contact>
                    <friction>
                      <ode>
                        <mu>100</mu>
                        <mu2>50</mu2>
                      </ode>
                      <torsional>
                        <ode/>
                      </torsional>
                    </friction>
                    <bounce/>
                  </surface>
                  <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                  <cast_shadows>0</cast_shadows>
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>100 100</size>
                    </plane>
                  </geometry>
                  <material>
                    <script>
                      <uri>file://media/materials/scripts/gazebo.material</uri>
                      <name>Gazebo/Grey</name>
                    </script>
                  </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
              </link>
            </model>
      """
 
    def _insertGroundPlane(self):
      self.file +="""
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      """
    def _defineBox(self,name,pos,rot,scale):
        self.file+=f"""
        <model name='{name}'>
              <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
              <link name='link'>
                <inertial>
                  <mass>1</mass>
                  <inertia>
                    <ixx>0.166667</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.166667</iyy>
                    <iyz>0</iyz>
                    <izz>0.166667</izz>
                  </inertia>
                  <pose>0 0 0 0 -0 0</pose>
                </inertial>
                <collision name='collision'>
                  <geometry>
                    <box>
                      <size>1 1 1</size>
                    </box>
                  </geometry>
                  <max_contacts>10</max_contacts>
                  <surface>
                    <contact>
                      <ode/>
                    </contact>
                    <bounce/>
                    <friction>
                      <torsional>
                        <ode/>
                      </torsional>
                      <ode/>
                    </friction>
                  </surface>
                </collision>
                <visual name='visual'>
                  <geometry>
                    <box>
                      <size>1 1 1</size>
                    </box>
                  </geometry>
                  <material>
                    <script>
                      <name>Gazebo/Grey</name>
                      <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                  </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
              </link>
                <static>1</static>
            </model>
        """
    def _insertBox(self,name,pos,rot,scale):
      self.file +=f"""
      <model name='{name}'>
        <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
        <scale>{scale[0]} {scale[1]} {scale[2]}</scale>
        <link name='link'>
          <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      """
    def _defineCylinder(self,name,pos,rot,scale):
        self.file += f"""
        <model name='{name}'>
              <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
              <link name='link'>
                <inertial>
                  <mass>1</mass>
                  <inertia>
                    <ixx>0.145833</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.145833</iyy>
                    <iyz>0</iyz>
                    <izz>0.125</izz>
                  </inertia>
                  <pose>0 0 0 0 -0 0</pose>
                </inertial>
                <collision name='collision'>
                  <geometry>
                    <cylinder>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cylinder>
                  </geometry>
                  <max_contacts>10</max_contacts>
                  <surface>
                    <contact>
                      <ode/>
                    </contact>
                    <bounce/>
                    <friction>
                      <torsional>
                        <ode/>
                      </torsional>
                      <ode/>
                    </friction>
                  </surface>
                </collision>
                <visual name='visual'>
                  <geometry>
                    <cylinder>
                      <radius>0.5</radius>
                      <length>1</length>
                    </cylinder>
                  </geometry>
                  <material>
                    <script>
                      <name>Gazebo/Grey</name>
                      <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                  </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
              </link>
                <static>1</static>
            </model>
        """
    def _insertCylinder(self,name,pos,rot,scale):
      self.file +=f"""
      <model name='{name}'>
        <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
        <scale>{scale[0]} {scale[1]} {scale[2]}</scale>
        <link name='link'>
          <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      """

    def _defineSphere(self,name,pos,rot,scale):
        self.file+=f"""
        <model name='{name}'>
              <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
              <link name='link'>
                <inertial>
                  <mass>1</mass>
                  <inertia>
                    <ixx>0.1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.1</iyy>
                    <iyz>0</iyz>
                    <izz>0.1</izz>
                  </inertia>
                  <pose>0 0 0 0 -0 0</pose>
                </inertial>
                <collision name='collision'>
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                  <max_contacts>10</max_contacts>
                  <surface>
                    <contact>
                      <ode/>
                    </contact>
                    <bounce/>
                    <friction>
                      <torsional>
                        <ode/>
                      </torsional>
                      <ode/>
                    </friction>
                  </surface>
                </collision>
                <visual name='visual'>
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                  <material>
                    <script>
                      <name>Gazebo/Grey</name>
                      <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                  </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
              </link>
                <static>1</static>
            </model>
        """
    def _insertSphere(self,name,pos,rot,scale):
      self.file +=f"""
      <model name='{name}'>
        <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
        <scale>{scale[0]} {scale[1]} {scale[2]}</scale>
        <link name='link'>
          <pose>{pos[0]} {pos[1]} {pos[2]} {radians(rot[0])} {radians(rot[1])} {radians(rot[2])}</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      """
    def _insertCamera(self):
      self.file+="""
      <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5.32643 -4.60432 2.17314 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
      """
    def _finilizeFile(self):
      self.file+="""
      </world>
      </sdf>
      """
    def _generateRandomName(self):
      letters = string.ascii_lowercase
      return ''.join(random.choice(letters) for i in range(5)) 

    def addBox(self,pos,rot=[0,0,0],scale=[1,1,1]):
      self.objects.append({
          "name":"box_"+self._generateRandomName(),
          "type":"box",
          "pos":pos,
          "rot":rot,
          "scale":scale
      })

    def addCylinder(self,pos,rot=[0,0,0],scale=[1,1,1]):
      self.objects.append({
          "name":"cylinder_"+self._generateRandomName(),
          "type":"cylinder",
          "pos":pos,
          "rot":rot,
          "scale":scale
      })

    def addSphere(self,pos,rot=[0,0,0],scale=[1,1,1]):
      self.objects.append({
          "name":"sphere_"+self._generateRandomName(),
          "type":"sphere",
          "pos":pos,
          "rot":rot,
          "scale":scale
      })

    def _defineObjects(self):
      self._defineLights()
      self._defineGroundPlane()
      self._defineConfig()
      for obj in self.objects:
        if obj["type"]=="box":
          self._defineBox(obj["name"],obj["pos"],obj["rot"],obj["scale"])
        if obj["type"]=="cylinder":
          self._defineCylinder(obj["name"],obj["pos"],obj["rot"],obj["scale"])
        if obj["type"]=="sphere":
          self._defineSphere(obj["name"],obj["pos"],obj["rot"],obj["scale"])
        
    def _insertObjects(self):
      self.file+=f"""
            <state world_name='{self.name}'>
        """
      self._insertSimConfig()
      self._insertGroundPlane()
      for obj in self.objects:
        if obj["type"]=="box":
          self._insertBox(obj["name"],obj["pos"],obj["rot"],obj["scale"])
        if obj["type"]=="cylinder":
          self._insertCylinder(obj["name"],obj["pos"],obj["rot"],obj["scale"])
        if obj["type"]=="sphere":
          self._insertSphere(obj["name"],obj["pos"],obj["rot"],obj["scale"])
      self._insertLights()
      self.file+= "</state>"
      self._insertCamera()
        
    def includeModel(self,name,pos,rot=[0,0,0],scale=[1,1,1]):
        self.file =+ "<include>\n<uri>model://{}</uri>\n<pose>{}</pose>\n<scale>{}</scale>\n</include>".format(
            name,
            "{} {} {} {} {} {}".format(*pos,*rot),
            "{} {} {}".format(*scale)
        )

    def _generateFile(self):
        self._beginFile()
        self._defineObjects()
        self._insertObjects()
        self._finilizeFile()
    
    def save(self):
      self._generateFile()
      output = open(self.path+self.name+".world","w")
      output.write(self.file)
      output.close()
      print( "file : "+ self.path+self.name+".world" + " has been created at "+self.path)
      self.generator.generateMap(self.objects)
