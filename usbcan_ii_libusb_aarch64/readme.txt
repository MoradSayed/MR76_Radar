https://manual.zlg.cn/web/#/55/2282 （有图文的在线文档，与下面的介绍一致）

USBCAN-II新版驱动基于libusb实现，请确保运行环境中有libusb-1.0的库。
如果是ubuntu，可连网在线安装，命令如下：
# apt-get install libusb-1.0-0

将libusbcan.so拷到/lib目录。
# sudo cp libusbcan.so /lib

运行C语言例程：
# sudo ./test

运行python例程：
# sudo python3 usbcan.py



设备调试常用命令：

1、查看系统是否正常枚举到usb设备，打印它们的VID/PID（USBCAN为0471:1200）：
	# lsusb

2、查看系统内所有USB设备节点及其访问权限：
	# ls /dev/bus/usb/ -lR

3、修改usb设备的访问权限使普通用户可以操作，其中xxx对应lsusb输出信息中的bus序号，yyy对应device序号：
	# chmod 666 /dev/bus/usb/xxx/yyy

4、如果要永久赋予普通用户操作USBCAN设备的权限，需要修改udev配置，增加文件：/etc/udev/rules.d/50-usbcan.rules，内容如下：
	SUBSYSTEMS=="usb", ATTRS{idVendor}=="0471", ATTRS{idProduct}=="1200", GROUP="users", MODE="0666"

	重新加载udev规则后插拔设备即可应用新权限：
	# udevadm control --reload
