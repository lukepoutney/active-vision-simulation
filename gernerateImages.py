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


obj_coord = [0,0,1]
def to_polar(cartesian_cam, cartesian_obj):
    #Get the polar coordinates of camera in relation to object
    x = cartesian_cam[0] - cartesian_obj[0]
    y = cartesian_cam[1] - cartesian_obj[1]
    z = cartesian_cam[2] - cartesian_obj[2]

    r = math.sqrt(x**2+y**2+z**2)
    theta = math.degrees(math.atan2(y,x))
    phi = math.degrees( math.atan2( ( math.sqrt(x**2+y**2) ),z ) )
    r = round(r, 2)
    theta = round(theta, 2)
    phi = round(phi, 2)
    return [r,theta,phi]


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
            basePosition=[0,0,1],
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
        obj_pos, _ = pb.getBasePositionAndOrientation(multiBodyId)
        #print(len(obj_pos))

        #import random #already imported
        import math


        #Generate coords for a given radius and total
        #radius = 2
        total = 50
        coords = []
        for i in range(0,total):
            theta = round( 2 * math.pi * random.random(), 2)
            phi = round( math.pi / 2 * random.random(), 2) #pi/2 as we are only looking for upper hemisphere, lower is obscured by table
            r = round(random.uniform(0.50,2.00), 2)
            
            print(r, theta, phi)

            x = r * math.sin(phi) * math.cos(theta)
            y = r * math.sin(phi) * math.sin(theta)
            z = r * math.cos(phi)  
            x,y,z = x+obj_pos[0], y+obj_pos[1], z+obj_pos[2]
            coords.append([x,y,z])
            #theta = round( math.degrees(theta), 2)
            #phi = round (math.degrees(phi), 2)

        projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=10.0)

        from PIL import Image  
        import PIL
        import numpy as np

        from PIL import Image

        #print("Coordinates: " + str(coords))
        i = 0

        for coord in coords:
            if coord[0]*coord[1] == 0 :
                upVector =[0,1,0]
            else:
                upVector=[0,0,1]
            #print(coord[0]*coord[1])
            viewMatrix = pb.computeViewMatrix(
                cameraEyePosition=[coord[0], coord[1], coord[2]],
                cameraTargetPosition=obj_pos,
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

            thewriter.writerow([str(shapeID)+'_%04d.png' %i, shapeID, to_polar(coord, obj_pos)])
            #rgbImg = rgbImg.save(str(i)+".jpg")
            i+=1
            #sleep(3)

        #wait = input("waiting")
        shapeID +=1
        pb.removeBody(multiBodyId)
        #sleep(1)
        #from time import sleep
        #sleep(100)