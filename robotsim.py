#!/bin/sh
"exec" "blender" "--python-exit-code" "1" "--python" "$0" "--" "$@"
import os, sys, bpy, math
print(bpy)
HERE = os.path.split(__file__)[0]
# Extract arguments safely
argv = sys.argv
print(argv)
if "--" in argv: script_args = argv[argv.index("--") + 1:]
else: script_args = []
print("script arguments:", script_args)
if not script_args: script_args = []  ## load a default robot arm

if 'Cube' in bpy.data.objects:
    bpy.data.objects['Cube'].scale = [100,100,1]
    bpy.data.objects['Cube'].location.z = -1

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
            s = False
            for n in skip:
                if obj.name.startswith(n):
                    print('skip:', obj.name)
                    s = True
                    break
            if s: continue
            print('loading:', obj.name)
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

def create_camera(name="camera", location=(0,0,0)):
    bpy.ops.object.camera_add()
    obj = bpy.context.active_object
    obj.name = name
    obj.rotation_euler.x = math.pi / 2
    if name == 'left': obj.rotation_euler.z = math.pi / 2
    elif name == 'right': obj.rotation_euler.z = -math.pi / 2
    elif name == 'back': obj.rotation_euler.z = math.pi
    obj.location = location
    obj.data.display_size = 0.2
    obj.data.lens = 16
    return obj


def quick_render(camera_obj, resolution_x=128, resolution_y=64, output_path="/tmp/blender_render.png"):
    """
    Renders the scene quickly using low-quality settings for speed.
    
    Args:
        camera_obj (bpy.types.Object): The camera object to render from.
        resolution_x (int): Horizontal resolution in pixels.
        resolution_y (int): Vertical resolution in pixels.
        output_path (str): File path for the output PNG image.
    """
    if not output_path.startswith('/tmp/'): output_path = '/tmp/' + output_path
    if not output_path.endswith('.png'): output_path += '.png'
    scene = bpy.context.scene
    # 1. Set the active camera for the scene
    scene.camera = camera_obj    
    # 2. Configure output resolution and format
    scene.render.resolution_x = resolution_x
    scene.render.resolution_y = resolution_y
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = output_path    
    # 3. Optimize settings for speed depending on the active render engine
    engine = scene.render.engine
    if engine == 'CYCLES':
        print('USING CYCLES')
        # Minimize Cycles samples and disable heavy features
        scene.cycles.samples = 1
        scene.cycles.preview_samples = 1
        scene.cycles.use_denoising = False
        scene.cycles.max_bounces = 0
        scene.cycles.diffuse_bounces = 0
        scene.cycles.glossy_bounces = 0
        scene.cycles.transmission_bounces = 0
        scene.cycles.volume_bounces = 0        
    elif engine == 'BLENDER_EEVEE' or engine == 'BLENDER_EEVEE_NEXT':
        print('USING EEVEE')
        # Eevee / Eevee Next speed optimizations
        scene.eevee.taa_render_samples = 1
        scene.eevee.use_bloom = False
        scene.eevee.use_ssr = False
        scene.eevee.use_gtao = False
        scene.eevee.use_motion_blur = False
    # 4. Disable anti-aliasing / pixel filter if applicable
    scene.render.film_transparent = False    
    # 5. Execute the render
    print(f"Starting quick render from camera '{camera_obj.name}' to {output_path}...")
    bpy.ops.render.render(write_still=True)
    print("Render complete!")
    return output_path

DEFAULT_ARM = os.path.join(HERE,'abb/irb120/irb120.blend')

class Robot:
    BOTS = []
    def __init__(self, size=(1,1,0.1), wheels=4, wheel_radius=0.1, arms=[DEFAULT_ARM]):
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

        self.cameras = { k : create_camera(k) for k in 'front back left right'.split() }
        for cam in self.cameras.values():
            cam.parent = self.camera_hub
            cam.location.z = 0.3

    def render_cameras(self):
        paths = []
        for cam in self.cameras.values():
            paths.append(
                quick_render(cam, output_path='.'.join( [self.root.name, cam.name]))
            )
        return paths


def main():
    for arg in script_args:
        if arg.endswith('.blend'):
            loaded = load_blend_objects(path)
            print(loaded)
        elif arg.endswith('.py'):
            py = open(arg).read()
            print('exec:',arg)
            exec(py)

if __name__=='__main__':
    main()
