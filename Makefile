default:
	./robotsim.py

headless:
	./headless.py

test:
	cd tests && ./render_test.py

test_anim:
	cd tests && ./render_anim_test.py

test_joints:
	cd tests && ./joint_control_test.py

test_drive:
	cd tests && ./drive_test.py

test_record:
	cd tests && ./record_test.py

test_all: test test_anim test_joints test_drive test_record

install:
	chmod +x robotsim.py
	chmod +x headless.py
	chmod +x tests/*.py
	sudo apt-get install blender
