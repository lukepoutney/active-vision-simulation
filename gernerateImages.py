import pybullet as pb
import pybullet_data

import os
import numpy as np
import math
import random
import time
import csv
import PIL
#import glob 
from time import sleep
from PIL import Image
from numpy import save

#Start the simulation and load in table
physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
tableId = pb.loadURDF("table/table.urdf")
pb.setGravity(0, 0, -9.8)
pb.setRealTimeSimulation(1)

#Setup the directories where images and CSVs will be stored
current_time = time.strftime("%Y%m%d-%H%M%S")
os.makedirs("imgs", exist_ok = True)
os.chdir("imgs")
os.mkdir(current_time)
os.chdir(current_time)
img_types = ["rgbImg","segImg"] #, "depthImg", "segImg"
for t in img_types:
    os.makedirs(t, exist_ok = True)
os.chdir("../..")

#shapes = ['000']
shapes = ['000','003','006','008','014','026']
r_range = [0.50,1.00]
obj_num = 10
img_size = 200

with open('imgs/'+current_time+'/rgbCSV.csv', 'w', newline='') as f:
    thewriter = csv.writer(f)
    thewriter.writerow(['Image Name', 'Shape Type', 'Coords'])

    shapeID = 0
    depth_imgs = []
    seg_imgs = []
    for shape in shapes:
        #Create the object
        visualShapeId = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName='procedural_objects/'+shape+'/'+shape+'.obj',
            rgbaColor=None,
            meshScale=[0.025, 0.025, 0.025])

        collisionShapeId = pb.createCollisionShape(
            shapeType=pb.GEOM_MESH,
            fileName='procedural_objects/'+shape+'/'+shape+'_coll.obj',
            meshScale=[0.025, 0.025, 0.025])

        multiBodyId = pb.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collisionShapeId, 
            baseVisualShapeIndex=visualShapeId,
            basePosition=[0,0,1],
            baseOrientation=pb.getQuaternionFromEuler([0, 0, 0]))

        textureId = pb.loadTexture('lead-gray.jpg')
        pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)

        #Allow object to fall and come to rest on table
        sleep(1)
        obj_pos, _ = pb.getBasePositionAndOrientation(multiBodyId)


        projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=10.0)

        #Generate coords for the given radius range and no of images per obj
        for i in range(0,obj_num):
            theta = round( 2 * math.pi * random.random(), 2)
            phi = round( math.pi / 2 * random.random(), 2) #pi/2 for hemisphere
            r = round(random.uniform(r_range[0],r_range[1]), 2)

            x = r * math.sin(phi) * math.cos(theta)
            y = r * math.sin(phi) * math.sin(theta)
            z = r * math.cos(phi)  
            x,y,z = x+obj_pos[0], y+obj_pos[1], z+obj_pos[2]

            #uncomment to have csv store theta and phi in degrees
            #theta = round( math.degrees(theta), 2)
            #phi = round (math.degrees(phi), 2)

            if phi == 0 :
                upVector =[0,1,0]
            else:
                upVector=[0,0,1]

            viewMatrix = pb.computeViewMatrix(
                cameraEyePosition=[x, y, z],
                cameraTargetPosition=obj_pos,
                cameraUpVector=upVector)

            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=img_size, 
                height=img_size,
                viewMatrix=viewMatrix,
                projectionMatrix=projectionMatrix)
                
            #print(np.array(rgbImg))
            rgb_array = np.array(rgbImg, dtype=np.uint8)
            rgb_array = np.reshape(rgb_array, (height, width, 4))
            rgb_array = rgb_array[:, :, :3]

            im = Image.fromarray(rgb_array)
            im.save('imgs/'+current_time+'/rgbIMG/'+str(shapeID)+'_%04d.png' %i)

            depth_array = np.array(depthImg)
            depth_array = depth_array.reshape(height, width)
            depth_imgs.append(depth_array)

            seg_array = np.array(segImg)
            seg_array = seg_array.reshape(height, width)
            seg_imgs.append(seg_array)

            seg_rgb_array = np.where(seg_array != 2, 0, rgb_array[:,:,0])
            seg_rgb_array = np.where(seg_array != 2, 0, rgb_array[:,:,1])
            seg_rgb_array = np.where(seg_array != 2, 0, rgb_array[:,:,2])

            seg_im = Image.fromarray(seg_rgb_array)
            seg_im.save('imgs/'+current_time+'/segIMG/'+str(shapeID)+'_%04d.png' %i)

            thewriter.writerow([str(shapeID)+'_%04d.png' %i, shapeID, [r,theta,phi]])

        shapeID +=1
        pb.removeBody(multiBodyId)

    depth_imgs = np.array(depth_imgs)
    save('imgs/'+current_time+'/depthImg.npy', depth_imgs)

    seg_imgs = np.array(seg_imgs)
    save('imgs/'+current_time+'/segImg.npy', seg_imgs)