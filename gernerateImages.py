import pybullet as pb
physicsClient = pb.connect(pb.GUI)

import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
import os
tableId = pb.loadURDF("table/table.urdf")

visualShapeId = pb.createVisualShape(
    shapeType=pb.GEOM_MESH,
    fileName='procedural_objects/000/000.obj',
    rgbaColor=None,
    meshScale=[0.05, 0.05, 0.05])

collisionShapeId = pb.createCollisionShape(
    shapeType=pb.GEOM_MESH,
    fileName='procedural_objects/000/000_coll.obj',
    meshScale=[0.05, 0.05, 0.05])

multiBodyId = pb.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collisionShapeId, 
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 1],
    baseOrientation=pb.getQuaternionFromEuler([0, 0, 0]))

import os, glob, random
#texture_paths = glob.glob(os.path.join('dtd', '**', '*.jpg'), recursive=True)
#random_texture_path = texture_paths[random.randint(0, len(texture_paths) - 1)]
#textureId = pb.loadTexture(random_texture_path)
#pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)

#Change texture
textureId = pb.loadTexture('lead-gray.jpg')
pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)

pb.setGravity(0, 0, -9.8)

pb.setRealTimeSimulation(1)

#To allow object time to Fall
from time import sleep
sleep(1)


#import random #already imported
import math


#Generate coords for a given radius and total
radius = 2
total = 100
coords = []
for i in range(0,total):
    a = random.random() * 2 * math.pi
    r = radius * random.random() #math.sqrt(random.random())
    x = r * math.cos(a)
    y = r * math.sin(a)
    z = round(math.sqrt( radius**2 - x**2 - y**2),2)
    x = round(x,2)
    y = round(y,2)
    coords.append([x,y,z])

#import os #already imported
import time
current_time = time.strftime("%Y%m%d-%H%M%S")
os.makedirs("imgs", exist_ok = True)
os.chdir("imgs")
os.mkdir(current_time)
os.chdir(current_time)
img_types = ["rgbImg", "depthImg", "segImg"]
for t in img_types:
    os.makedirs(t, exist_ok = True)

projectionMatrix = pb.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=10.0)

from PIL import Image  
import PIL
import numpy as np

from PIL import Image

print("Coordinates: " + str(coords))
i = 0
for coord in coords:
    viewMatrix = pb.computeViewMatrix(
        cameraEyePosition=[coord[0], coord[1], coord[2]],
        cameraTargetPosition=[0, 0, 1],
        cameraUpVector=[0, 0, 1])

    width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
        width=224, 
        height=224,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix)
    
    print(np.array(rgbImg))
    rgb_array = np.array(rgbImg, dtype=np.uint8)
    rgb_array = np.reshape(rgb_array, (height, width, 4))
    rgb_array = rgb_array[:, :, :3]
    
    im = Image.fromarray(rgb_array)
    im.save('rgbIMG/%04d.png' %i)
    #rgbImg = rgbImg.save(str(i)+".jpg")
    i+=1
    #sleep(3)

wait = input("waiting")
#from time import sleep
#sleep(100)