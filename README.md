## 丛林环境行军道路探测

### 指标要求
（１）行军道路判别准确度：大于等于50%．


### 测试方法

将一个可行道路分成ｎ段，手控无人机沿着这条路行走，得到重建的效果后，计算正确率．

### 技术要求
（１）无人机在地势高度多变的丛林稳定飞行，基于机载视觉传感器获得的点云数据，对周围的环境进行网格重建．

（２）找到网格中的人可行道路．


## 结果展示

（１）在西北工业大学自动化学院楼后面的小土坡进行测试，该地地势起伏较大，障碍物较少．黄色为可行道路，彩色为局部地图，为不确定区域．

![image01](https://github.com/liuzhenboo/mav_find_road/blob/master/img/01.png)

（２）在西北工业大学海天苑启翔楼附近的小树林，此地地势起伏小，障碍物多．黄色为可行道路，红色为树木，白色为局部地图即不确定区域．

![image02](https://github.com/liuzhenboo/mav_find_road/blob/master/img/02.png)

（３）在西北工业大学海天苑启翔楼对面的山上测试，该地地势变化极大．红色为障碍物，黄色为可行军道路．

![image03](https://github.com/liuzhenboo/mav_find_road/blob/master/img/03.png)

![image04](https://github.com/liuzhenboo/mav_find_road/blob/master/img/04.png)
