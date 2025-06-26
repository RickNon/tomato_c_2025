
# tomato-dynamixel

dynamixelサーボモータを動作させる際に用いるパッケージです。

## how to use

```bash
# AXモータ用のテスト(ID1が回転)
$ roslaunch tomato_dynamixel ax_pos.launch
```
```bash
# AX用のテスト(ID1と2が回転)
$ roslaunch tomato_dynamixel ax_double_pos.launch
```
```bash
# MX用のテスト(ID10が回転)
$ roslaunch tomato_dynamixel mx_pos.launch
```
```bash
# joy+AX用のテスト(ID1がコントローラで回転)
$ roslaunch tomato_dynamixel joy_ax.launch
```
```bash
# joy+AX+MX用のテスト(ID1およびID10がコントローラで回転)
$ roslaunch tomato_dynamixel joy_ax_mx.launch
```
```bash
# joy+AX+MX+XC用のテスト(ID1およびID10,ID20がコントローラで回転)
$ roslaunch tomato_dynamixel joy_ax_mx_xc.launch
```
```bash
# XC用のテスト(ID20が回転)
$ roslaunch tomato_dynamixel xc_pos.launch
```
