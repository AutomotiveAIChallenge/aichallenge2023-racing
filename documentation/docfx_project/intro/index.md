# 自動運転AIチャレンジ2023 (シミュレーション) イントロダクション

## About Competition

&emsp;皆様には下記の流れに沿って、本大会に取り組んでいただきます。  

1. 与えられたシナリオのクリアに向け、Autoware.universeをベースに自動運転ソフトウェアを開発
2. ローカル環境で手順1で開発したソフトウェアを検証  
3. 開発したソフトウェアを、提出用ページから評価システムにアップロード  
4. アップロード後、シミュレーションに基づき評価システムがあなたのソフトウェアのスコアを算出します。
    ※順位は開催期間中に記録したスコアのうち、一番高いスコアに基づきます。
    （オンライン環境へのご案内は後日行います）  

## About Autoware

&emsp; AutowareとはROS2を使用したオープンソースの自動運転ソフトウェアです。LiDARやカメラなどからデータを取得するセンシング機能、センシングデータを組み合わせて車両の位置を推定するLocalization機能などがモジュールとして存在し、それらが相互に連携することで、自動運転を実現しています。本ソフトウェアは日本国内の公道での走行実験の実績もあります。  
&emsp; 本大会ではAutowareの中でも研究・開発向けディストリビューションであるAutoware.universeを使用します。その他のディストリビューションやAutowareのこれまでの開発の流れについては、[こちら](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/difference-from-ai-and-auto/)を御覧ください。
  
## About AWSIM

&emsp;AWSIMはUnity上で動作するオープンソースの自動運転用シミュレータになります。ROS2 nativeに対応していること、WindowsやUbuntuに対応していることから、誰でも簡単に自動運転アルゴリズムのシミュレーションを行うことが可能です。
&emsp;AutowareでAWSIMを活用した場合、AutowareのノードがAWSIMからセンシングデータをSubscribeし、受け取ったデータを各モジュールで処理を行い、その結果（車両制御情報）をAWSIMにPublishすることで、AWSIM上の車両を制御します。詳細は[こちら](https://github.com/tier4/AWSIM)をご確認ください。
 ![awsim](../images/intro/awsim.png)

## Related Documentations

* [自動運転Aiチャレンジ公式HP](https://www.jsae.or.jp/jaaic/)
* [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)
* [AWSIM](https://github.com/tier4/AWSIM)