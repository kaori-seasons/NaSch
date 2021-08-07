# NaSch
- 为满足2015年交通流建模国赛B题的要求,基于双车道的matlab版元胞自动机改进

参考
-  四川大学Milky Zhang的[A Two-Lane Cellular Automaton Traffic Flow Model with the Keep-Right Rule](http://www.wanfangdata.com.cn/details/detail.do?_type=perio&id=wlxb201108057)

>介绍

从wiki上搜到的Cellular_automaton定义如下:
>细胞自动机由*单元*格的规则网格组成，每个*单元格*处于有限数量的*[状态之一中](https://en.wikipedia.org/wiki/State_(computer_science))*，诸如*开*和*关*（与[耦合的地图网格](https://en.wikipedia.org/wiki/Coupled_map_lattice)相反）。网格可以是任何有限数量的维度。对于每个单元格，称为其*邻域的*一组单元格相对于指定的单元格被定义。通过为每个单元分配状态来选择初始状态（时间*t*= 0）。一个新的*一代*，创建（前进*吨*由1），根据一些固定*规则*（通常数学函数）根据单元格的当前状态和其邻域中单元格的状态来确定每个单元格的新状态。[通常，更新单元状态的规则对于每个单元是相同的，并且不随时间改变，并且同时应用于整个网格，尽管已知例外，诸如随机单元自动机](https://en.wikipedia.org/wiki/Stochastic_cellular_automaton)和[异步单元自动机](https://en.wikipedia.org/wiki/Asynchronous_cellular_automaton)。

NaSch模型建立的初衷，是为了解决自敏感性交通流车辆拥堵的特点,采用184规则
#184号元胞自动机定义如下:
车辆行驶规则为:黑色元胞表示被一辆车占据
白色表示无车,若前方各自有车,则停止。若前方为空则前进一格,第t时刻到t_1时刻车辆元胞的变化图解如下

![5309010-ae3007850c38018a.png](http://upload-images.jianshu.io/upload_images/5309010-3cb31a2c2218b99d.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

上图仅显示了t时刻到t+1时刻车辆元胞的变化过程,现在我们将变化过程逐步分解，称之为演化规则

从第t时刻车辆的位置至t+1时刻车辆的演化过程如下:
a)加速过程:Vn->(Vn+1,Vmax)

b)安全刹车过程:Vn->(Vn-1,dn-1)

c)随机变化过程Vn ->max (Vn-1,0)

d)位置更新:Xn ->Xn+Vn

dn =Xn+1 -Xn -L(L为车辆长度)



到这里，各位新人可能看的还是一脸懵逼，结合国赛中实现的[三车道换道规则](https://github.com/complone/NaSch)，下面我们依次解释一下各个演化规则的意义

> 1)加速:司机总期望以最大的速度行驶
2)安全刹车:为避免与前车发生碰撞
3)随机慢化(即车辆行驶时不确定因素)
- 过度刹车
- 道路条件变化
- 心里因素
- 延迟因素
4)位置：车辆前进

####当Vmax=2时

![5309010-ae3007850c38018a.png](http://upload-images.jianshu.io/upload_images/5309010-0b4cb0b6dfe1f492.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

我们引入下面四个参数，表示单位时间内交通流通过横断面流量

- probc;          % 车辆的密度

- B;      %单位时间内车流所占用的道路宽度

- probslow;      % 随机慢化的概率

- Dsafe;            % 表示换道事车至少与后面车距离多少个单位才算安全

下面,start making program

模型建立:
```matlab
close all;

B=3;            %The number of the lanes

plazalength=50;  %The length of the simulating highways

h=NaN;          %h is the handle of the image

[plaza,v]=create_plaza(B,plazalength);

h=show_plaza(plaza,h,0.1);

iterations=1000;    % 迭代次数

probc=0.1;          % 车辆的密度

probv=[0.1 1];      % 两种车流的密度分布

probslow=0.3;      % 随机慢化的概率

Dsafe=1;            % 表示换道车至少与后面车距离多少个单位才算安全

VTypes=[1,2];      %道路上一共有几种最大速度不同的车辆,速度是什么

[plaza,v,vmax]=new_cars(plaza,v,probc,probv,VTypes);%一开始就在车道上布置车辆，做周期循环驾驶，也方便观察流量密度之间的关系

size(find(plaza==1))

PLAZA=rot90(plaza,2);

h=show_plaza(PLAZA,h,0.1);
```
#from 模型 to 编程  

之前有讲过，元胞自动机是一个动态生成过程

高速公路可以看成一个网格状视图,车辆上格子颜色产生变动时,就有车辆消失或生成在边界(该元胞已经死亡或者生存),我们需要在车辆不断更新的状态实时显示在图形界面上,

模型转换的思想就浅显易懂了,设置两个车流，一个为正在更新的车流密度,一个为希望此时高速公路能达到的车流密度，其实我们只需要设置两种车流的密度,因为在更新句柄的过程中,车流的相对密度是相对不变的，详细会在之后的create_plaza函数中说明

probv=[0.1 1];      % 两种车流的密度分布

这就要涉及到matlab的GUI函数式编程思想,简略介绍一下:所谓GUI就是一种用户界面图形化操作，我们所使用qq,MSN的聊天界面就属于GUI,但凡GUI都会有一个活动句柄,每当我们要对窗体就行操作的时候，要更新窗体句柄的活动状态,通知窗体要对他进行怎样的改变。

先设计句柄更新后的状态，引入new_cars.m函数

该函数模型的车道变换规则如下:

(1) 如果vmax>gap,且gapleft≥gap，则从右车道变换至左车道。

(2) 如果 vmax

(3) 如果vback

如果vright>gapleft，则vright=gapleft(禁止右车道的车辆超过左车道车辆)。

old为正在进行仿真的车流密度,entry为希望此时通过横断面的车流密度
```matlab
function new = new_cars(B, L, old, entry)

new = old;

if entry > 0

if entry <= L

x = randperm(L);

y = ceil((B-L)/2+1);

for i = 1:entry

new(1, (y + x(i))) = 1;

end

end

if entry > L

y = ceil((B-L)/2+1);

for i = 1:L

new(1,(y + i)) = 1;

end

end

end
```
