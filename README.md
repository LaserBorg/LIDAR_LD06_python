# LIDAR_LD06_python
This code can use  Lidar's LD06 (LDS06) provided by LDROBOT from python. and It displays the acquired point cloud in real time in matplotlib.



pip install pyserial

fix tty rights:
https://github.com/esp8266/source-code-examples/issues/26
sudo usermod -a -G dialout your-username
sudo usermod -a -G tty your-username




# How to use
1. Clone this repository and change `Serial(port='/dev/tty.usbserial-0001'...)` in main.py to your own port (LDRobot LD06 on Windows: 'COM6').
2. Run `pip install -r requirements.txt` in venv environment.
3. Run `python main.py`.
4. Press the E key to exit.

# About LD06(LDS06)
- Sales page https://www.inno-maker.com/product/lidar-ld06/
- Datasheet http://wiki.inno-maker.com/display/HOMEPAGE/LD06

# LICENSE
Please see [LICENSE](https://github.com/henjin0/LIDAR_LD06_python_loder/blob/main/LICENSE).
