# 果蝇轨迹分析

本程序对果蝇飞行轨迹进行分析

## nearest

检测两只果蝇飞到紧邻点，统计前后飞行状态

## collision(crash)

通过速度预测果蝇飞行轨迹，如果在一定时间后发生碰撞，取出进行进一步研究

### collision部分代码运行方法及顺序

1. 获取潜在碰撞点
        
        首先将轨迹数据导入workspace后，运行pre_process_crash.m。
        该脚本将清洗原始数据，去除短轨迹，并检测可能出现的碰撞（A对很多B），将其存入trackerW的Bs字段

2. 获取策略点

        使用脚本strategy_detector_multi.m脚本检测发生较大机动的策略点。
        通过修改threshold_XXX_min和threshold_XXX_max调整响应的最大值和最小值。

        结果保存在trackerW.Bs中：
        若其中time_strategy不为-1,则其值为策略点时间；
        若time_strategy为-1则说明果蝇没有较大机动

3. 绘制统计曲线
        
        分别使用draw_multi_Bs.m draw_multi_Bs_less_than_epsilon.m对 2 中的结果进行可视化。
        将产生fig和png图片，默认保存在上上层目录的statistics文件夹中（与src目录同级）。
        使用前必须先创建该文件夹。

        存在策略点的结果中，蓝色五角星代表发现未来可能碰撞的时间点T，红色六角星代表策略点

        不存在策略点的结果中，蓝色五角星代表发现未来可能碰撞的时间点T，红色五角星代表期望的碰撞时间T+N


4. 绘制轨迹
        
        使用draw_multi_trace.m 绘制轨迹，默认保存在上上层目录的trace文件夹中。

        图中符号说明：
        红色轨迹为A果蝇，蓝色轨迹为B果蝇。
        六角星是轨迹起始点，红色+号为检测到碰撞的时间T
        红色菱形为预期碰撞点，A轨迹上的红色圆圈是2中阈值条件下的策略点

