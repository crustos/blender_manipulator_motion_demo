#!/bin/sh
"exec" "blender" "--python-exit-code" "1" "--python" "$0" "--" "$@"
import os, sys, bpy
print(bpy)

# Extract arguments safely
argv = sys.argv
if "--" in argv: script_args = argv[argv.index("--") + 1:]
else: script_args = []
print("script arguments:", script_args)
if not script_args: script_args = ['abb/irb120/irb120.blend']  ## load a default robot arm

if 'Cube' in bpy.data.objects:
    bpy.data.objects['Cube'].scale.z = 0

def load_blend_objects(path, link=False, skip=['Camera', 'Plane']):
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
    # Create the base primitive cube (Blender's default primitive size is 2.0)
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    cube_obj = bpy.context.active_object
    cube_obj.name = name
    cube_obj.scale = size    
    # Optional: Apply the scale so the object's base scale resets to (1, 1, 1)
    bpy.ops.object.transform_apply(scale=True)    
    return cube_obj

def main():
    for arg in script_args:
        if arg.endswith('.blend'):
            loaded = load_blend_objects(arg)
            print(loaded)
            root = create_empty('ROOT')
            tip = None
            for ob in loaded:
                if ob.name.startswith("Tool tip"): tip = ob
                if not ob.parent:
                    ob.parent = root
            assert tip
            tip.parent = root
        
            
if __name__=='__main__':
    main()
