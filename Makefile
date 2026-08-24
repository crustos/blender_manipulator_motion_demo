default:
	./headless.py

headed:
	./robotsim.py

install:
	chmod +x robotsim.py
	chmod +x headless.py
	sudo apt-get install blender
