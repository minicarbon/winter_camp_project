# winter_camp_project
我是Xilinx 2021冬令营HLS强化班的陈桂荣。
这是Xilinx 2021冬令营的项目，目标是实现一个多通道卷积，通过脉动阵列实现性能优化，并希望能将卷积最终在tiny_yolo_v2算法进行目标检测。
希望在这个过程中能通过这一个算法来对深度学习进一步了解，同时契合于目前的科研方向，利于后期的科研。
整体的计划是在8号到25号这段时间，对多通道的卷积层进行设计和性能优化，然后使用脉动阵列进行改进。
性能改进完成后进行仿真，生成IP核，在pynq_z2 板子进行硬件实现。至于其他层（池化、激活等）的设计，具体需要看卷积层进度完成。2月8日。



由于能力和技术问题，直到现在，已经完成对tiny_yolo_ip的设计，整个算法有九层（其中包括卷积，池化激活等），由于文件较大，目前只上传部分源码。仿真部分只完成了一部分，而且目前还没有部署到板子上进行实现。后续会继续完成仿真和板子上的实现。最终的目标是想完成搭建一个SOC的系统。2.21


本以为是25号截止，后面没上传SDK.cpp的代码，现在更新了项目，增加了SDK的部分，本来目标是在PYNQ-z2上实现的，但是实验室的师兄对于ZYNQ的7020更熟悉，后面就在SKD上面写控制程序。--3.5
