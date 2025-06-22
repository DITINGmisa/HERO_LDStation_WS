项目基于TensorRT8.5开发，需要首先安装TensortRT。

下载本项目，编译运行。

```bash
# 安装本项目
git clone https://github.com/DITINGmisa/HERO_LDStation_WS.git
cd HERO_LDStation_WS
catkin_make
```

infer下会有5个可执行程序，分别为：

+ car_utils 原始的检测车的demo，还可以用来转换模型
+ armor_utils 原始的检测装甲的demo，还可以用来转换模型
+ car_demo 封装好的检测车的demo，只可以直接用来检测文件夹
+ armor_demo 封装好的检测装甲的demo，只可以直接用来检测文件夹
+ infer 总的推理程序，会进行顺序的检测车和装甲板

首先把scripts文件夹下的gen_wts.py复制到yolov5的根目录下。

![](https://cdn.nlark.com/yuque/0/2023/png/22969079/1690444284863-73a5347d-198e-4b97-8f9e-9ed039c87215.png)

生成wts文件。

```bash
python gen_wts.py -w path_to_pt_model -o path_to_wts_model
```

将wts文件移动到本功能包的model文件夹下。

![](https://cdn.nlark.com/yuque/0/2023/png/22969079/1690444737509-a927f073-2a75-4995-8f52-a5b4edc6a28e.png)

首先转换模型，我们的模型分别是640*640输入，batch_size为2，class_num为4，<font style="color:rgb(38, 38, 38);">128*128输入，batch_size为4，class_num为12</font>。如果模型参数不同，或者想修改推理时的<font style="color:rgb(38, 38, 38);">batch_size，</font>需要修改yololayer_[xxx].h，yolov5_[xxx].h，[xxx]_infer.h和infer.cpp来适应你的模型。

```bash
rosrun infer armor_utils -s [path to wts] [path to engine] [model type]
```

其中model type有：[n/s/m/l/x/n6/s6/m6/l6/x6 or c/c6 gd gw]，例：

```bash
rosrun infer armor_utils -s src/infer/model/yolov5s_armor_v3.wts src/infer/model/yolov5s_armor_v3.engine s
```

随后对模型进行检验。

```bash
rosrun infer armor_utils -d [path to engine] [path to imgDir]
```

也可以使用demo程序进行检测，目前仅实现了检测功能，可以后续自行拓展。

```bash
rosrun infer car_demo [path to engine] [path to imgDir] [batch_size] [num_classes]
```

![](https://cdn.nlark.com/yuque/0/2023/jpeg/22969079/1690446327359-94be17c8-ccb8-4cb3-93d7-5d3ee4409564.jpeg)

对于infer，car的检测器加入了跨类别NMS，armor则只会输出置信度最高的框。参数则通过rosParam传入，可以通过launch文件修改。

![](https://cdn.nlark.com/yuque/0/2023/png/22969079/1690447431439-9cfe8c1b-1609-473f-865a-1a68c775547f.png)

