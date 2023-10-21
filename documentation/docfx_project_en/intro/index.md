# Japan Automotive AI Challenge 2023 (Simulation) Introduction

## About Competition

&emsp; You are expected to follow the following steps to participate in the competition.

1. Develop software based on Autoware.universe to complete the given scenario  
2. Validate the software created in step 1 in the local environment  
3. Upload the validated software through the submission page
4. Upon your submission, the evaluation system will execute a simulation and calculate your score based on the simulation results
    *The ranking will be determined based on the highest score you achieved during the competition period.
    (Invitation to the submission page will be provided at a later date)  

## About Autoware

&emsp; Autoware is an open-source self-driving software that uses ROS2, a sensing function that acquires data from LiDAR and cameras, and a localization function that estimates the location of the vehicle by combining the sensing data as modules, The modules work in conjunction with each other to realize automated driving. This software has been tested on public roads in Japan.  
&emsp; This competition will use Autoware.universe, which is a distribution for research and development within Autoware. For more information on other distributions and Autoware's development process to date, see [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/difference-from-ai-and-auto/).
  
## About AWSIM

 &emsp;AWSIM is an open source self-driving simulator that runs on Unity, ROS2 native, and supports Windows and Ubuntu, making it easy for anyone to simulate self-driving algorithms.
  When utilizing AWSIM in&emsp;Autoware, Autoware nodes subscribe to sensing data from AWSIM, process the received data in each module, and publish the results (vehicle control information) to AWSIM to The vehicle is controlled on AWSIM. For more information, please check [here](https://github.com/tier4/AWSIM).
 ![awsim](../../images/intro/awsim.png)

## Related Documentations

* [Official website of the Automated Ai Challenge](https://www.jsae.or.jp/jaaic/)
* [Autoware.universe](https://github.com/autowarefoundation/autoware.universe)
* [AWSIM](https://github.com/tier4/AWSIM)