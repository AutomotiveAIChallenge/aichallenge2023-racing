# Local Enviroment

<br>

<!-- > [!REGISTER]
> Register from here!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br> -->

 &emsp; Participants will be asked to create a ROS2 package to carry out the scenario, and the following ROS2 package is provided in aichallenge2023-racing/docker/aichallenge/aichallenge_ws/src as sample code to serve as a base for this in this repository The following ROS2 package is provided.  
 &emsp; aichallenge_launch, sim_msgs change is not applied at submission

## Sample Code (Help Wanted)
 &emsp;aichallenge2023-racing/docker/aichallenge/aichallenge_ws/src directory structure is below
* aichallenge_launch
  * 大元のlaunchファイルaichallenge.launch.xmlを含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
* aichallenge_submit
  * 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。
  * aichallenge_submit_launch.launch.xmlが大元のlaunchファイルaichallenge.launch.xmlから呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
  * autoware_launch, autoware_universe_launch
    * Autowareのlaunch, config関連のパッケージをコピーして一部編集しています。Dockerイメージ内のAutowareにはここに含まれているパッケージが削除されています。こちらを編集することでAutowareの動作の変更を行うことが出来ます。
    * 改変前のファイルを利用したい場合は、[autoware_launch](https://github.com/autowarefoundation/autoware_launch/tree/awsim-stable), [autoware_universeのlaunchディレクトリ](https://github.com/autowarefoundation/autoware.universe/tree/awsim-stable/launch)をご利用ください。
  * autoware_micro 最小構成のautowareを作成するためのフォルダです。
  * dallara_interface Autonomaで使用している車両に適合するためのパッケージ群です。
  * sample_pakcages カスタムパッケージ作成のためのsampleのパッケージ群です。  

### Steps for Execution

1. Docker Image Build
```
#In the aichallenge2023-racing directory
cd docker/train
bash build_docker.sh
```

2. Docker Container Run
```
#In the aichallenge2023-racing directory
cd docker/train
bash run_container.sh
```

3. Code Build
```
# In the Rocker container
cd /aichallenge
bash build_autoware.sh
 ```
 4. Start AWSIM  
Start AWSIM by referring to [Setup page](../setup/index.html).

5. Sample Code Run
 ```
# In the Rocker container
cd /aichallenge
bash run.sh
```
 &emsp; If setup is successful, rviz will display a point cloud map and begin automatic operation.
 
 ### Customizing Autoware

 Ways in which Autoware can be customized, or new packages be added into Autoware, are explained in the [Customizing Autoware](../customize/index.html) page.
