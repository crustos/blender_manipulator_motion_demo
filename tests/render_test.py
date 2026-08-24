#!../headless.py
print('hello render test...')

def test1():
    from random import uniform
    robots = [Robot(), Robot()]
    for r in robots:
        r.root.location.x = uniform(-5,5)
        r.root.location.y = uniform(-5,5)
        r.root.rotation_euler.z = uniform(-3,3)

    for r in robots:
        pngs = r.render_cameras()
        print(pngs)
        for png in pngs:
            assert os.path.isfile(png)


test1()
