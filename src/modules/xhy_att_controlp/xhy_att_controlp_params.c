//定义了Rattitude模式下，油门的阈值
PARAM_DEFINE_FLOAT(XHY_RATT_TH, 0.8f);

//定义了位置控制，非线性动态逆的两个控制参数，Am1是角度环，Am2是角速率环
PARAM_DEFINE_FLOAT(AM1_OUTER, 2.7f);

PARAM_DEFINE_FLOAT(AM2_OUTER, 8.0f);

//定义飞行器的三轴转动惯量的先验知识，单位为国际单位kg*m^2
PARAM_DEFINE_FLOAT(IXX, 0.025f);

PARAM_DEFINE_FLOAT(IYY, 0.07f);

PARAM_DEFINE_FLOAT(IZZ, 0.022f);

//定义比例控制器的自适应增益，这个太大了会导致方程变为刚性，从而必须用很小的步长才能求解，但是实际飞行这个是不是可以比较大
PARAM_DEFINE_FLOAT(ADAPTIVE_GAIN, -300.0f);


//定义低通滤波器的系数，也反应了带宽的选取
PARAM_DEFINE_FLOAT(LOWPADDFILTER, 30.0f);

//下面这三个变量反应了这个公式，其实就是kg -inv(inv(Am2)*inv([Ixx 0 0;0 Iyy 0;0 0 Izz]))
PARAM_DEFINE_FLOAT(KG1, 0.2f);

PARAM_DEFINE_FLOAT(KG2, 0.04f);

PARAM_DEFINE_FLOAT(KG3, 0.176f);



//定义和控制分配有关的几个变量
PARAM_DEFINE_FLOAT(CONTROL_AL1, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL2, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL3, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL4, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL5, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL6, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL7, 0.0f);
PARAM_DEFINE_FLOAT(CONTROL_AL8, 0.0f);
