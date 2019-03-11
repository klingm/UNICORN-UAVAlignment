How to Test this code:
1. (optional) sudo python <path>/unicorn/Adafruit_python_gpio/setup.py install
2. check with ls -l to make sure python scripts are exceutable
3. sudo python test_docker.py <cmd>
	- cmd:
		a1 - align (45 rpm rate)
		r  - release uav and reset to stowe position
		rh - release hold (turn off motors)
		h  - halt current command
