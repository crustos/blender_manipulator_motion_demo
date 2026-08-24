default:
	./robotsim.py

headless:
	./headless.py

install:
	chmod +x robotsim.py
	chmod +x headless.py
	sudo apt-get install blender
