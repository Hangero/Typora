# USB摄像头及其驱动

**USB**，是英文Universal Serial Bus（通用串行总线）的缩写，是一个[外部总线](https://baike.baidu.com/item/外部总线/1250422?fromModule=lemma_inlink)标准，用于规范[电脑](https://baike.baidu.com/item/电脑/124859?fromModule=lemma_inlink)与[外部设备](https://baike.baidu.com/item/外部设备/3741161?fromModule=lemma_inlink)的连接和[通讯](https://baike.baidu.com/item/通讯/396194?fromModule=lemma_inlink)。是应用在[PC](https://baike.baidu.com/item/PC/107?fromModule=lemma_inlink)领域的接口技术。

**USB摄像机**是采用[USB接口](https://baike.baidu.com/item/USB接口/493294?fromModule=lemma_inlink)的安防摄像机，即插即用，傻瓜式操作，无需采集卡，无需电源，免拆机箱、支持笔记本电脑。与传统的[监控摄像机](https://baike.baidu.com/item/监控摄像机/2540417?fromModule=lemma_inlink)相比成本更低，而且可以远程网络观看。方便实用，操作简单。

**UVC全称为USB Video Class，即：[USB](https://baike.baidu.com/item/USB/99797?fromModule=lemma_inlink)视频类**，是一种为USB视频捕获设备定义的协议标准。是Microsoft与另外几家设备厂商联合推出的为USB视频捕获设备定义的协议标准，已成为USB org标准之一。

如今的主流操作系统(如Windows XP SP2 and later, Linux 2.4.6 and later, MacOS 10.5 and later)都已提供UVC设备驱动，因此符合UVC规格的硬件设备在不需要安装任何的驱动程序下即可在主机中正常使用。使用UVC技术的包括摄像头、数码相机、类比影像转换器、电视棒及静态影像相机等设备。

最新的UVC版本为UVC 1.5，由USB Implementers Forum定义包括基本协议及负载格式。

[网络摄像头](https://baike.baidu.com/item/网络摄像头/4506781?fromModule=lemma_inlink)是第一个支持UVC而且也是数量最多的UVC设备，操作系统只要是 Windows XP SP2 之后的版本都可以支持 UVC，当然 Vista 就更不用说了。Linux系统自2.4以后的内核都支持了大量的设备驱动，并可以支持UVC设备。

使用 UVC 的好处 USB 在 Video这块也成为一项标准了之后，硬件在各个程序之间彼此运行会更加顺利，而且也省略了驱动程序安装这一环节。

**libuvc是一个用于USB视频设备的跨平台库,**支持对导出标准USB视频类(USB Video Class, UVC)接口的USB视频设备进行细粒度控制，使开发人员能够为以前不受支持的设备编写驱动程序，或仅以通用方式访问UVC设备。

