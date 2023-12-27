# SetUp

<br>

> [!REGISTER]
> こちらから参加登録!
> [https://www.jsae.or.jp/jaaic/en/index.php](https://www.jsae.or.jp/jaaic/en/index.php)

<br>

## Minimum Hardware Requirements
本大会で使用していただくPCの動作環境として以下を推奨しております。

* OS: Ubuntu 22.04
* CPU: Intel Corei7 (8 cores) or higher
* GPU: NVIDIA Geforce RTX 1060 or higher
* Memory: 32 GB or more
* Storage: SSD 30 GB or higher

上記のスペックを満たすPCをご用意できない方は、下記の「PC2台で参加する方向け」のスペックをご参照ください。
### 2台のPCを使用する方向け
#### Autoware PC
* OS: Ubuntu 22.04
* CPU: Intel Corei7 (4 cores) or higher
* GPU: NVIDIA Geforce GTX 1060 or higher
* Memory: 16 GB or higher
* Storage: SSD 10 GB or higher
* 詳細は[こちら](https://autowarefoundation.github.io/autoware-documentation/main/installation/)

#### AWSIM PC
* OS: Ubuntu 22.04
* CPU: Intel Corei7 (4 cores and 8 threads) or higher
* GPU: NVIDIA Geforce RTX 1060 or higher

※Autoware動作PCとAWSIM動作PCは、同じネットワーク内に配置してください。
配置できていれば、基本的には追加設定をすることなく、PC間のトピック通信は可能です。万が一、トピック通信ができなかった場合はファイアーウォールの解除、もしくはルールの見直しをお願いします。
  

### CPU only のNotePCを使用される方向け
#### Minimum Hardware Requirements for Headless mode
本大会で使用していただくPCの動作環境としてサポートを十分に提供できない可能性がありますが、
今後より多くの方にコンテストに参加し、活発なディスカッションを行っていただくために、
試験的に以下のCPUのみの構成のPCのみでも参加可能になるようにしました。

* OS: Ubuntu 22.04
* CPU: Intel Core i7-8650U (4 cores) with Intel HD Graphics (no NVIDIA GPUs)
* Memory: 16 GB or more
* Storage: SSD 16 GB or higher

- other CPU only cases

* CPU : AMD Ryzen 5 5600G with Radeon Graphics
* Memory : 32GB

* CPU AMD® Ryzen 7 pro 4750u with radeon graphics × 16
* Memory 32.0 GiB

* DELL XPS 13 9300
* CPU: Intel(R) Core(TM) i7-1065G7
* Memory: 16GB
    
## Environment Setup
### AWSIM(Ubuntu)
#### 事前準備
* (CPU onlyの方またはすでにNVIDIA Driverがはいっている方はskip) Nvidiaドライバのインストール
  1. リポジトリの追加、update、install
  ```sh
  sudo add-apt-repository ppa:graphics-drivers/ppa
  sudo apt update
  sudo ubuntu-drivers autoinstall
  ```
  2. 再起動の後、下記コマンドを実行し、インストールできていることを確認
  ```
  nvidia-smi
  ```
  ![nvidia-smi](../images/setup/nvidia-smi.png)
 
 * Vulkunのインストール
    1. パッケージリストの更新
    ```sh
    sudo apt update
    sudo apt install libvulkan1
    ```
 * コースの準備
   1. [GoogleDrive](https://drive.google.com/drive/folders/1EjgBxB_x0_xRla7_FdPaeEkiOU3vxW7e)から最新の`AWSIM.zip`をダウンロードし、`aichallenge2023-racing/docker/aichallenge`内に大会用AWSIM実行ファイルを展開
   2. パーミッションを図のように変更    
   ![パーミッション変更の様子](../images/setup/permmision.png)  
   3. ファイルをダブルクリックで起動
   4. GPU版のAWSIMは下記のような画面が表示されることを確認
      (CPU版のAWSIMは画面の描画なし)
      ![awsim](../images/setup/awsim.png)

### Dockerの事前準備  
下記のインストールをお願いします。
  * [docker](https://docs.docker.com/engine/install/ubuntu/)
  * [rocker](https://github.com/osrf/rocker) 
     * Dockerコンテナ内のRviz、rqtなどのGUIを使用するために用います。
  * [git lfs](https://packagecloud.io/github/git-lfs/install)

以下のコマンドでdocker rocker gitlfsがinstallされます
```sh
# docker インストールの下準備
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
# 最新のDockerをインストール
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
# インストールできているかをテスト
sudo docker run hello-world

# rocker install
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt-get install python3-rocker

# gitlfs install
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
```

  * (CPU onlyの方はskip)[Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
  下記でinstallお願いします。
```sh
# インストールの下準備
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# インストール
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

#インストールできているかをテスト
sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi

#最後のコマンドで以下のような出力が出れば成功です。
#（下記はNVIDIAウェブサイトからのコピペです）


# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 450.51.06    Driver Version: 450.51.06    CUDA Version: 11.0     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
# | N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
# |                               |                      |                  N/A |
# +-------------------------------+----------------------+----------------------+
# +-----------------------------------------------------------------------------+
# | Processes:                                                                  |
# |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
# |        ID   ID                                                   Usage      |
# |=============================================================================|
# |  No running processes found                                                 |
# +-----------------------------------------------------------------------------+

```



* Dockerイメージの準備・起動 〜 Autowareの準備
   1. Dockerイメージを入手
   ```
   docker pull ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda
   ```
   user groupに自分のアカウントを追加
   ```
   sudo usermod -aG docker $USER
   ```
   以下のコマンドでgroupsにdockerが追加されていることを確認
   ```
   groups $USER
   # $ USERNAME : USERNAME adm cdrom sudo ... docker
   ```
   ここまで確認できたら再度loginし直す
   
   docker imagesで以下のような表示が見えていることを確認
    ```
    REPOSITORY                                                                       TAG                                 IMAGE ID       CREATED          SIZE
    ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda   latest                              9601fc85f1bd   3 weeks ago      7.31GB    
    ```

    2. 大会用データのダウンロード
    ```
    sudo apt install -y git-lfs
    git lfs clone https://github.com/AutomotiveAIChallenge/aichallenge2023-racing
    ```
    3. 大会用dockerイメージのビルド
    ```
    cd aichallenge2023-racing/docker/train
    bash build_docker.sh
    ```
    4. 大会用dockerコンテナの起動
    
    GPU環境の方は以下
    ```
    bash run_container.sh
    ```
    CPUのみの環境の方は以下
    ```
    bash run_container_cpu.sh
    ```
#### Dockerコンテナ内でのAWSIM起動
DockerコンテナからAWSIMを起動したい場合は、Dockerイメージの準備手順(前述)に従ってDockerイメージを導入した後、以下の手順で行ってください。
  1. `aichallenge2023-racing/docker/aichallenge`内に大会用AWSIM実行ファイルを展開(以下、`aichallenge2023-racing/docker/aichallenge/AWSIM/AWSIM.x86_64`に配置されているものとします。)
  2. rockerを起動
   新たにterminalを開いて`docker image ls`で以下のようなimageが存在していることを確認してください。
   ```
REPOSITORY                                                                      TAG                                 IMAGE ID       CREATED          SIZE
aichallenge-train                                                               latest                              67a4d45d119d   16 minutes ago   7.37GB
   ```
   確認ができたら以下のコマンドでrockerを起動してください。  
   
   GPU環境の方は以下
   ```
    cd ./aichallenge2023-racing/docker/train
    bash run_container.sh
   ```
   CPUのみの環境の方は以下
   ```
    cd ./aichallenge2023-racing/docker/train
    bash run_container_cpu.sh
   ```
   新たに開いたterminalで`docker images` で以下のようにdocker が存在していることを確認してください。
   ```
REPOSITORY                                                                      TAG                                 IMAGE ID       CREATED          SIZE
aichallenge-train                                                               latest                              67a4d45d119d   16 minutes ago   7.37GB   
```
  1. コンテナ内で以下を実行
   ```
    cd /aichallenge
    bash run_awsim.sh 
   ```

> [!NOTE]
> AWSIMからpublish・subscribeされているトピックのメッセージは，一部`/aichallenge/aichallenge_ws/src/sim-msgs`で定義されています．これらのメッセージを扱うには，以下のコマンドを実行してください．
> ```
> cd /aichallenge
> bash build_autoware.sh
> source /aichallenge/aichallenge_ws/install/setup.bash 
> ```

### 地図データosmの配置 WIP

本大会ではlanelet2_mapファイルの改変を可としました。
`/aichallenge2023-racing/docker/aichallenge/aichallenge_ws/src/aichallenge_submit/aichallenge_submit_launch/map/*`に配置して、ファイル構成が以下になるように配置してください。
```
aichallenge2023-racing
└ docker
 └ aichallenge
  └ AWSIM
  └ aichallenge_ws/src/aichallenge_submit/aichallenge_submit_launch
     └map
      └lanelet2_map.osm
```
※ 経路計画を行う際にlanelet2_map参照して行わないといけないなどの制約もございません。（csvファイルを参照し経路を生成することも可とします。）
※ PCDファイルは入れていますが、今回提出していただく必要はございません。Mapの編集に必要な場合のみご使用ください。

### Autoware      
 * Autowareの動作確認  
   AWSIMを用いて、Autowareの動作確認を行う方法を記します。
   1. AWSIMを起動
   2. Autowareを起動
   ```
   # Rockerコンテナ内で
   cd /aichallenge
   bash build_autoware.sh
   bash run_autoware.sh
   ```
   3. 下記のような画面(Rviz2)が表示され，車両の自動走行が開始されることを確認．
   ![autoware](../images/setup/autoware.png)

### 2回目以降の起動
1. AWSIMの起動
   1. コンテナの起動
   ```
    cd ./aichallenge2023-racing/docker/train
    bash run_container.sh または bash run_container_cpu.sh
   ```
   2. コンテナ内部でAWSIMの起動
   ```
    cd /aichallenge
    bash run_awsim.sh 
   ```
2. Autowareの起動
   1. コンテナの起動
   ```
    cd ./aichallenge2023-racing/docker/train
    bash run_container.sh または bash run_container_cpu.sh
   ```
   2. 必要に応じてコンテナ内部でAutowareのbuild
   ```
    cd /aichallenge
    bash build_autoware.sh 
   ```
   3. コンテナ内部でAutowareの起動
   ```
    cd /aichallenge
    bash run_autoware.sh
   ```
### 画面録画の方法

デフォルトのrockerではディスプレイドライバを自分のPCに設定できていないため以下のオプションを追加する必要があります。
```
#!/bin/bash
rocker --device /dev/dri --x11 --user ... # CPU版
rocker --device /dev/dri --nvidia --x11 --user ... # GPU版
```

### 変更点の取り込み

#### dockerのupdate
```sh
docker pull ghcr.io/automotiveaichallenge/aichallenge2023-racing/autoware-universe-no-cuda
```

#### repositoryのupdate
```sh
cd aichallenge2023-racing # path to aichallenge2023-racing
git pull origin/main
```