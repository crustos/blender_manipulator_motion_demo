default:
	./robotsim.py

headless:
	./headless.py

test:
	cd tests && ./render_test.py

install:
	chmod +x robotsim.py
	chmod +x headless.py
	chmod +x tests/*.py
	sudo apt-get install blender
