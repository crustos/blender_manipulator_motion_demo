#!../headless.py
print('hello render anim test...')
from random import uniform, random
from PIL import Image

robots = [Robot(), Robot()]
for r in robots:
    r.root.location.x = uniform(-5,5)
    r.root.location.y = uniform(-5,5)
    r.root.rotation_euler.z = uniform(-3,3)

Frames = {}

@RobotSim
def callback():
    for bot in robots:
        if bot.root.name not in Frames: Frames[bot.root.name] = []
        bot.root.location.x += uniform(-0.2,0.2)
        bot.root.location.y += uniform(-0.2,0.2)
        if random() * random() < 0.3: bot.root.rotation_euler.z = uniform(-0.2,0.2)
        pngs = bot.render_cameras()
        print(pngs)
        views = []
        Frames[bot.root.name].append(views)
        for png in pngs:
            assert os.path.isfile(png)
            img = Image.open(png)
            views.append(img)
            
    if RobotSim.ticks >= 10:
        RobotSim.stop()
        ## TODO OUTPUT ANIMATION
        ## take all the camera view frames for each bot, first bot top, second bot bottom, and all frames


