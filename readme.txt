I have made very slight modifications to the code and made it in oops style.

changes made:
1. made it to objects/ the main code is kept intofunction from external code.
2. changed variable runstepper.halt to halt
3. changed motorpairs to [(1,2)] , so even while calling main function motorpairs[0] is passed in 
both thread[0] and thread[1]
4. thread.join() is blocking call added

I have tested it on rpi in here , I don't have active i2c port. so i made sure rest of code works

if you have any code changes to code on github, please feel free to commit those changes to attached code.

Send me a revised code if necessary, ill write a docking module in ros, which can call both unicorn and charging module tx.

How to Test this code:
1. (optional) sudo python <path>/unicorn/Adafruit_python_gpio/setup.py install
2. check with ls -l to make sure python scripts are exceutibles
3. sudo python test_docker.py a1
