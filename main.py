from world_creator import GazeboWorldCreator

if __name__=="__main__":
  
    creator = GazeboWorldCreator("testworld","/home/gemy/world creator/",[200,200])
    creator.addBox([0,-10,0],[0,0,0],[20,1,10])
    creator.addBox([0,10,0],[0,0,0],[20,1,10])
    creator.addBox([10,0,0],[0,0,90],[21,1,10])
    creator.addBox([-10,0,0],[0,0,90],[21,1,10])
    creator.save()