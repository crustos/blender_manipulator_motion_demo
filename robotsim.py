#!/bin/sh
"exec" "blender" "--python-exit-code" "1" "--python" "$0" "--" "$@"
import os, sys, bpy, math
print(bpy)

# Extract arguments safely
argv = sys.argv
if "--" in argv: script_args = argv[argv.index("--") + 1:]
else: script_args = []
print("script arguments:", script_args)
if not script_args: script_args = []  ## load a default robot arm

if 'Cube' in bpy.data.objects:
    bpy.data.objects['Cube'].scale.z = 0

def load_blend_objects(path, link=False, skip=['Camera', 'Plane', 'Camera.001']):
    """
    Links or appends all objects from a .blend file into the active scene.
    Returns a dictionary mapping the file path to a list of loaded object references.
    """            
    file_objects = []
    # Open the external library and target its objects
    with bpy.data.libraries.load(path, link=link) as (data_from, data_to):
        # Assigning the list tells Blender to load all these objects.
        # Blender automatically resolves and imports dependencies (armatures, materials).
        data_to.objects = data_from.objects
    
    # The objects are now in bpy.data, but not yet visible in the scene.
    # We must link them to the active scene collection.
    for obj in data_to.objects:
        if obj is not None:
            if obj.name in skip: continue
            if obj.name not in bpy.context.scene.collection.objects:
                bpy.context.scene.collection.objects.link(obj)
            file_objects.append(obj)
            if obj.type=='ARMATURE':
                print('ARMATURE:', obj.name)
                obj.pose.ik_solver = "LEGACY"  ## force standard IK solver
            
    return file_objects


def create_empty(name="Empty"):
    """
    Creates a new Empty object, links it to the active scene collection, 
    and returns the object reference.
    """
    # Create a new empty data-block
    empty_data = bpy.data.objects.new(name, None)
    
    # Set the empty display type if desired (e.g., 'PLAIN_AXES', 'CUBE', 'SPHERE')
    empty_data.empty_display_type = 'PLAIN_AXES'
    
    # Link the empty object to the current scene's collection
    bpy.context.scene.collection.objects.link(empty_data)
    
    return empty_data

def create_cube(name="Cube", size=(1,1,1), location=(0.0, 0.0, 0.0)):
    """
    Creates a new mesh cube with individual X, Y, Z dimensions, 
    links it to the active scene collection, and returns the object reference.
    """
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    cube_obj = bpy.context.active_object
    cube_obj.name = name
    cube_obj.scale = size    
    # Optional: Apply the scale so the object's base scale resets to (1, 1, 1)
    bpy.ops.object.transform_apply(scale=True)    
    return cube_obj

def create_cylinder(name="Cylinder", radius=0.1, depth=1, location=(0,0,0)):
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=depth)
    obj = bpy.context.active_object
    obj.name = name
    obj.rotation_euler.y = math.pi / 2
    bpy.ops.object.transform_apply(rotation=True)
    obj.location = location
    return obj

class Robot:
    BOTS = []
    def __init__(self, size=(1,1,0.1), wheels=4, wheel_radius=0.1, arms=['abb/irb120/irb120.blend']):
        Robot.BOTS.append(self)
        self.root = create_empty('ROBOT.ROOT')
        self.body = create_cube('ROBOT.BODY', size )
        self.body.parent = self.root
        self.arms = []
        for path in arms:
            assert path.endswith('.blend')
            loaded = load_blend_objects(path)
            print(loaded)
            root = create_empty('ARM.ROOT')
            tip = None
            for ob in loaded:
                if ob.name.startswith("Tool tip"): tip = ob
                if not ob.parent: ob.parent = root
            assert tip
            tip.parent = root
            ## TODO allow custom placement of multiple arms, for now we just put all the arms in the center front.
            root.location = size
            root.location *= 0.5
            root.location.x = 0
            #root.location.y -= 0.25
            root.location.z += 0.05
            root.rotation_euler.z = math.pi / 2
            root.parent = self.root
            self.arms.append(root)

        self.wheels = []
        x,y,z = size
        if wheels == 4:  ## TODO others, support 1, 2, 3, and wheels with custom placement
            wheel = create_cylinder('W.L.REAR', radius=wheel_radius, depth=wheel_radius, location=(-x/2,-y/2,-z/2))
            wheel.parent = self.root; self.wheels.append(wheel)
            wheel = create_cylinder('W.L.FRONT', radius=wheel_radius, depth=wheel_radius, location=(-x/2,y/2,-z/2))
            wheel.parent = self.root; self.wheels.append(wheel)

            wheel = create_cylinder('W.R.REAR', radius=wheel_radius, depth=wheel_radius, location=(x/2,-y/2,-z/2))
            wheel.parent = self.root; self.wheels.append(wheel)
            wheel = create_cylinder('W.R.FRONT', radius=wheel_radius, depth=wheel_radius, location=(x/2,y/2,-z/2))
            wheel.parent = self.root; self.wheels.append(wheel)

        self.root.location.z = wheel_radius * 1.5

        self.front_hub = create_cube('FRONT.HUB', size=(0.4,0.2,0.1), location=(0,y/2,0.05))
        self.front_hub.parent = self.root

        self.rear_hub = create_cube('REAR.HUB', size=(0.4,0.2,0.1), location=(0,-y/2,0.05))
        self.rear_hub.parent = self.root

        self.camera_hub = create_cube('CAM.HUB', size=(0.18,0.18,0.5))
        self.camera_hub.location.y = -y / 2
        self.camera_hub.location.z = 0.25
        self.camera_hub.parent = self.root

def main():
    for arg in script_args:
        if arg.endswith('.blend'):
            loaded = load_blend_objects(path)
            print(loaded)
    ## default Robot
    Robot()
            
if __name__=='__main__':
    main()
