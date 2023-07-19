# car的标注数据, class,conf,x0,y0,x1,y1 
string cls      # 类别，格式为 car_[red/blue]_[number] or car_[unknown]
float32 conf    # 置信度
float32 x0      # 左上角x坐标
float32 y0      # 左上角y坐标
float32 x1      # 右下角x坐标
float32 y1      # 右下角y坐标