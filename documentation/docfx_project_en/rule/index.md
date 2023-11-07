# Rule

This page explains the rules and ranking system for the competition. Please note that the contents of this page may change during the tournament period.

## Ranking System

The ranking of participants is determined based on the following two metrics:

1. Distance Points:
   1. For participants who could not reach the goal within the time limit, their ranking is determined by the `distance points (0-100)` corresponding to the distance they could travel from the starting point within the time limit.
   2. If the participant's vehicle completed a lap within the time limit, the `distance points` are 100.
   3. In the case of disqualification, the `distance points` are 0.
   4. If the distance points are the same, the person with less time added due to penalties will rank higher.
2. Total Time:
   1. For participants who reached the goal within the time limit, their ranking is determined by the `total time`, which is the sum of the time elapsed from start to finish and penalty time. The penalty time will be calculated based on violations during the race.  

***Ranking Example***

| Distance Points | Time from Start to Goal | Time Added Due to Penalties | Total Time | Rank | 
| --------------- | ----------------------- | --------------------------- | ---------- | ---- |
| 100             | 01:10                   | 00:00                       | 01:10      | 1    |
| 100             | 01:30                   | 00:10                       | 01:40      | 2    |
| 100             | 01:20                   | 00:30                       | 01:50      | 3    |
| 50              | N/A                     | 00:00                       | N/A        | 4    |
| 50              | N/A                     | 00:10                       | N/A        | 5    |
| 10              | N/A                     | 00:00                       | N/A        | 6    |

### Penalty

Deviation from the course and collisions with other vehicles running alongside are considered violations. There are 3 types of penalty for violations:

1. Disqualification
2. 10-second addition to the driving time
3. 5-second addition to the driving time

The following lists each violation and the corresponding penalty.

**Critical Violation (Disqualification)**

- Deviating from the course (for more than 2 seconds)
- Deviating from the course (more than 5m from the course boundary)
- Driving in the wrong direction on the course

**Sevior Violation (Penalty: 10 seconds per occurrence):**

- Colliding with another vehicle (if the difference in relative speed with the other vehicle is more than 5[m/s] **OR** the collision state lasts for more than 2 seconds)

**Minor Fouls (Penalty: 5 seconds per occurrence):**

- Deviating from the course (for less than 2 seconds **AND** less than 5m from the course boundary)
- Colliding with another vehicle (if the difference in relative speed with the other vehicle is less than 5[m/s] **AND** the collision state is resolved within 2 seconds)
- Overtaking while off course

**Note**

- In cases where another vehicle collides with the rear of your vehicle (e.g., being rear-ended by another vehicle), it is not considered the fault of your vehicle and no penalty will be applied.
- If overtaking occurs while off course, both the deviation from the course (a minor foul) and overtaking while off course (a minor foul) will apply.

## Submission

Participants are requested to upload the software they developed to the evaluation system via the submission page. Three simulations will be conducted for each upload, and distance points and total time will be calculated for each simulation. The best score out of the three simulation results will be reflected in the ranking.

### How to check results

The details and results of the simulations are planned to be available for confirmation through a json file output by the evaluation system and videos of the simulations. Please wait for further announcements.