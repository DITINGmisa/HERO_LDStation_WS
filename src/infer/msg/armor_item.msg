# armor的标注数据,idx,class,conf,x,y,w,h
# R1: 1, R2: 2, R3: 3, R4: 4, R5: 5, R6: 6
# B1: 8, B2: 9, B3: 10, B4: 11, B5: 12, B6: 13
# R0：-2， B0：-1
uint8 idx    # armor的所属的车的id
int8 cls     # armor的类型
float32 conf # armor的置信度
float32 x    # armor的中心点x坐标
float32 y    # armor的中心点y坐标
float32 w    # armor的宽度
float32 h    # armor的高度
