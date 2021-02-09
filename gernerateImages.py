import pybullet as pb
physicsClient = pb.connect(pb.GUI)

import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
import os
tableId = pb.loadURDF("table/table.urdf")

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
os.chdir("../..")



#shape = '026'
#shape_no = '5'
shapes = ['000','003','006','008','014','026']
import csv

with open('imgs/'+current_time+'/rgbCSV.csv', 'w', newline='') as f:
    thewriter = csv.writer(f)
    thewriter.writerow(['Image Name', 'Shape Type', 'Coords'])

    shapeID = 0
    for shape in shapes:
        print(shape) 

        visualShapeId = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName='procedural_objects/'+shape+'/'+shape+'.obj',
            rgbaColor=None,
            meshScale=[0.05, 0.05, 0.05])

        collisionShapeId = pb.createCollisionShape(
            shapeType=pb.GEOM_MESH,
            fileName='procedural_objects/'+shape+'/'+shape+'_coll.obj',
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
        sleep(2)


        #import random #already imported
        import math


        #Generate coords for a given radius and total
        radius = 2
        total = 500
        coords = []
        for i in range(0,total):
            a = random.random() * 2 * math.pi
            r = radius * random.random() #math.sqrt(random.random())
            x = r * math.cos(a)
            y = r * math.sin(a)
            z = round(math.sqrt( radius**2 - x**2 - y**2)+1,2)
            x = round(x,2)
            y = round(y,2)
            coords.append([x,y,z])

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
            if coord[0]*coord[1] == 0 :
                upVector =[0,1,0]
            else:
                upVector=[0,0,1]
            print(coord[0]*coord[1])
            viewMatrix = pb.computeViewMatrix(
                cameraEyePosition=[coord[0], coord[1], coord[2]],
                cameraTargetPosition=[0, 0, 1],
                cameraUpVector=upVector)


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
            im.save('imgs/'+current_time+'/rgbIMG/'+str(shapeID)+'_%04d.png' %i)

            thewriter.writerow([str(shapeID)+'_%04d.png' %i, shapeID, coord])
            #rgbImg = rgbImg.save(str(i)+".jpg")
            i+=1
            #sleep(3)

        #wait = input("waiting")
        shapeID +=1
        pb.removeBody(multiBodyId)
        #sleep(1)
        #from time import sleep
        #sleep(100)