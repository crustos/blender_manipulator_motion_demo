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
            img.load()  # Forces Pillow to read the data stream immediately
            views.append(img.convert("RGB"))
            
    if RobotSim.ticks >= 12:
        RobotSim.stop()
        bot_names = list(Frames.keys())
        if len(bot_names) < 2:
            print("Error: Expected 2 robots.")
            return

        bot1_name, bot2_name = bot_names[0], bot_names[1]
        bot1_frames = Frames[bot1_name]
        bot2_frames = Frames[bot2_name]

        num_frames = min(len(bot1_frames), len(bot2_frames))
        combined_frames = []

        for i in range(num_frames):
            imgs1 = bot1_frames[i]
            imgs2 = bot2_frames[i]
            w, h = imgs1[0].size
            # Create canvas
            canvas_width = w * 4
            canvas_height = h * 2
            canvas = Image.new("RGB", (canvas_width, canvas_height))
            # Paste Robot 1 views on top
            for j, img in enumerate(imgs1):
                canvas.paste(img, (j * w, 0))
            # Paste Robot 2 views on bottom
            for j, img in enumerate(imgs2):
                canvas.paste(img, (j * w, h))
            # Convert canvas to palette mode ("P") for optimal GIF compliance
            canvas_p = canvas.convert("P", palette=Image.ADAPTIVE, colors=64)
            combined_frames.append(canvas_p)

        # Save as an animated GIF
        if combined_frames:
            output_gif = "/tmp/robots_animation.gif"
            combined_frames[0].save(
                output_gif,
                save_all=True,
                append_images=combined_frames[1:],
                duration=200,
                loop=0
            )
            print(f"Animation saved successfully as {output_gif}")



