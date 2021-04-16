import pybullet as pb
physicsClient = pb.connect(pb.GUI)

import pybullet_data
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF('plane.urdf')
import os
tableId = pb.loadURDF("table/table.urdf")

shapes = ['000','003','006','008','014','026']
for i in shapes: #shapes
    #i = "%03d" % i
    #i = str(i)
    visualShapeId = pb.createVisualShape(
        shapeType=pb.GEOM_MESH,
        fileName='procedural_objects/'+i+'/'+i+'.obj',
        rgbaColor=None,
        meshScale=[0.025, 0.025, 0.025])

    collisionShapeId = pb.createCollisionShape(
        shapeType=pb.GEOM_MESH,
        fileName='procedural_objects/'+i+'/'+i+'_coll.obj',
        meshScale=[0.025, 0.025, 0.025])

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
    from time import sleep
    wait = input("waiting")
    if wait == 'q':
        break
    pb.removeBody(multiBodyId)