# blender script to make a helical tire


import bpy, bmesh
import math

LAYER = 0.32
RADIUS = 55.0
HEIGHT = 30.4
ANGLE = 22.5 * math.pi / 180.0
TIRE = 'Tire'

output = []


def deselect():
    #bpy.ops.object.select_all(action='DESELECT')
    for i in bpy.data.objects:
        i.select_set(False)

# select & activate the object
def selectByName(name):
    deselect()
    for i in bpy.context.scene.objects:
        #print(i.name)
        if i.name == name:
            i.select_set(True)
            bpy.context.view_layer.objects.active = i
            return i
    print("getByName didn't find %s" % (name))


cube = bpy.ops.mesh.primitive_cube_add(scale=[RADIUS * 2, RADIUS * 2, LAYER])
cube_obj = bpy.context.object

layers = int(HEIGHT / LAYER)
#layers = 5
for i in range(0, layers):
    print('layer=' + str(i) + ' / ' + str(layers))
# position the cube
    z = i * LAYER + LAYER / 2
    cube_obj.location = [0, 0, z]

# duplicate the tire
    tire = selectByName(TIRE)
    tire2 = tire.copy()
    tire2.data = tire.data.copy()
    bpy.context.collection.objects.link(tire2)
    
# rotate the model
    tire2.rotation_euler = [0, 0, ANGLE * i / layers]
    bool = tire2.modifiers.new(type="BOOLEAN", name="bool")
    bool.object = cube_obj
    bool.operation = 'INTERSECT'
    bool.use_self = True
    
    deselect()
    tire2.select_set(True)
    bpy.context.view_layer.objects.active = tire2
    bpy.ops.object.convert(target="MESH") 

    output.append(tire2)








