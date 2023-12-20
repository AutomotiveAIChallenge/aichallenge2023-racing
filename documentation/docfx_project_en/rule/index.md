Certainly! Here's the translation of the provided text in the same format:

# Rule

&emsp;This page explains the rules and ranking system of the competition. Please note that the contents of this page may change during the competition period.

## Ranking System

&emsp;Participants' rankings are determined based on the following two criteria:

1. Distance Points:
   1. For participants who cannot reach the goal within the time limit (10 minutes), the ranking is determined by `Distance Points (0-5700)`, based on the distance traveled from the start point within the time limit.
   2. If the goal is reached within the time limit, the `Distance Points` are approximately 5700 points. (Details of the exact distance are omitted for brevity.)
2. Total Time:
   1. For participants who reach the goal within the time limit, the ranking is determined by the `Total Time`, which includes the time taken from start to goal plus any penalties.

***Example of Ranking***

| Distance Points | Time from Start to Goal | Time Addition due to Penalty | Total Time | Rank | 
| --------------- | ----------------------- | ---------------------------- | ---------- | ---- |
| 5700            | 01:10                   | 00:00                        | 01:10      | 1    |
| 5700            | 01:30                   | 00:10                        | 01:40      | 2    |
| 5700            | 01:20                   | 00:30                        | 01:50      | 3    |
| 60              | N/A                     | 00:00                        | N/A        | 4    |
| 50              | N/A                     | 00:10                        | N/A        | 5    |
| 10              | N/A                     | 00:00                        | N/A        | 6    |

### Goal Position
- The completion of the run (reaching the goal) is when the following goal points are crossed.
```yaml
  goal.position.x: 21912.0
  goal.position.y: 52267.5
```
- A goal can be set by using goal_pose_setter.

### Penalty

&emsp;Violations such as deviating from the course or colliding with other vehicles are penalized. The penalties for violations are:

1. End of run
2. Addition of 10 seconds to the running time
3. Addition of 5 seconds to the running time

Below are the violations and their corresponding penalties:

**End of Run**

- Deviating more than 5m from the course boundary for more than 10 seconds
- Being more than 100 seconds away from the course boundary

**Major Violation (Penalty: 10 seconds/occurrence):**

- Collision with another vehicle that lasts more than 2 seconds
- Deviating more than 5m from the course boundary
- Deviating more than 2 seconds from the course boundary

**Minor Violation (Penalty: 5 seconds/occurrence):**

- Deviating from the course (less than 2 seconds **and** less than 5m from the course boundary)
- Collision with another vehicle (resolved within 2 seconds)

**Note**

- In cases where another vehicle collides with the rear of your vehicle (e.g., rear-end collision by another vehicle), it is not considered your fault, and no penalty is applied.
- Do not drive in the wrong direction.
- Do not cut corners.

## Submission

&emsp;Participants are required to upload their developed software to the evaluation system via the submission page. Three simulations will be conducted for each upload, and distance points and total time calculations will be performed for each simulation. The highest score among the three simulation results will be reflected in the rankings.

### How to Check Results

&emsp;The score of the results will be sent to `result.json`.
#### Log Format for Results
&emsp;The results will be output in the following format in `~/awsim-logs/result.json`.

```json
{
  "rawLapTime": 72.77926,
  "distanceScore": 457.0,
  "lapTime": 302.779266,
  "isLapCompleted": false,
  "isTimeout": false,
  "trackLimitsViolation": [
    19, # out of track less than 2 sec
    19, # out of track more than 2 sec
    2,  # out of track less than 5m
    2,  # out of track more than 5m
    0   # not used
  ],
  "collisionViolation": [
    0, # collision less than 2 sec
    0, #

 collision more than 2 sec
    0, # not used
    0  # not used
  ]
}
```

&emsp;Additionally, `~/awsim-logs/verbose_result.json` will also be output in the following format.

```json
{
  "rawLapTime": 0.0,
  "distanceScore": 0.0,
  "lapTime": 0.0,
  "isLapCompleted": false,
  "isTimeout": false,
  "boundsViolations": [
    {
      "distance": 0.3017645,
      "distanceFromBound": 2.26600266,
      "duration": 0.0160293132
    },
    {
      "distance": 2.776487,
      "distanceFromBound": 1.01412094,
      "duration": 0.0801174641
    },
    {
      "distance": 2.91162729,
      "distanceFromBound": 1.1498549,
      "duration": 0.08674298
    },
    ....
  ]
  "collisionViolations": []
}
```