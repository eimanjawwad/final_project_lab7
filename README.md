# 🏀 AirBUD: Basketball Utility Dog 🏀

### Eiman Jawwad, Felicity Huang, Gavin Griffin

![img](/Pupper.png)

This repo was forked from our implementation of Lab 7 of CS123. Original lab repo located [here](https://github.com/cs123-stanford/lab_7_fall_2025)

## [Presentation slides (demo video included)](https://docs.google.com/presentation/d/17IBkJ8fFBrybfbrVzmjYj0ddUGKrWw0dGcLruVsvTPk/edit?usp=sharing)

## Added/Modified Files
* lab_7.py - Object tracking file. Modified to stop and return to Idle state before tracked object and center on it.
* tracking_status.txt - Written/read to to communicate whether tracking portion has ended
#### pupper_llm/karel/
* karel.py - Added AIM_UP/AIM_MIDDLE joint positions and controller switching logic (neural/joint position). Added functions for aiming up, aiming middle (default standing pose), and press_trigger to shoot basketball.
* karel_realtime_commander.py - Added parsing for "shoot" command. Calls aim up, aim middle, press_trigger functions in karel.py
* karel_testing.py - Test file for functions in karel.py
* shooting_mech.py - Sends high voltage to GPIO pin 26 to fire launcher then low to start up flywheels
## Hardware
* Informed primarily by weight and size constraints, our launcher went through many iterations. We initially thought to use a Nerf gun; later we found out a previous group had already implemented this, and the battery-powered guns we were interested in were too cumbersome. Our next step was to design an original flywheel launcher, designed around Nerf Rival foam balls (~0.875" diameter). You can see our final CAD at: [here](https://cad.onshape.com/documents/c921c09dcc6de268eb584d5f/w/4800d4ed8bfc96cc9374761e/e/c9d1d93adb94f1d6f68957f0?renderMode=0&uiState=693e650e5411e6b191a60ee5).  
* Time constraints prevented the use of flyback diodes to prevent high voltage loads across our transistors, but they held up quite well for the demo. See our circuit diagram [here](./circuit_diagram.jpeg), and our Arduino code [here](./launcher_arduino.ino).



## Testing
To test functions in karel.py: 
First ros launch in one terminal,
```
ros2 launch lab_7.launch.py
```
Then run karel_test.py
```
python3 pupper_llm/karel/karel_test.py
```

## Running full system
```
bash scripts/run_full_system.sh
```
This will activate the full system, where you can use the microphone to command Pupper. By saying "Pupper, shoot a basketball", you'll activate the shooting sequence, where Pupper tracks a designated object (default: stop sign), stops a certain distance away from it, bends down, and shoots. To ensure the OpenAI voice API works, ensure that you first setup a valid API key.



