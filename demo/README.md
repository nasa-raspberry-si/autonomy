# Demo for the adaptation case 1 and case 2
After following the [steps](https://github.com/nasa-raspberry-si/autonomy/tree/ow9#run-the-raspberry-si-autonomy) to run the [mission1.txt](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/evaluation/mission1.txt), we trigger an adaptation during different lander arm operations by randomly setting an arm fault parameter (e.g., scoop_yaw_joint_locked_failure) in [the fault configuration tab](https://github.com/nasa/ow_simulator/wiki/Fault-Injection-and-Modeling/7d6438354885c32fea86fc294a62bef33eb0a18c) in rqt (developed by [OceanWATERS](https://github.com/nasa/ow_simulator)). In this demo, we differentiate an arm fault into two groups based on when it occurs. In the first group, the arm fault happens during an Grind operation and causes it to be aborted. We call it a digging failure. In the second group, the arm fault happens during arm operations other than the Grind operation.

All tests below were conducted in one run. The recording of the experiment can be found [here](https://pjamshid-nas.us6.quickconnect.to/vs/sharing/nT3s55MM#!bW92aWUtMjY=).

## Adaptation Case 1: Failure caused by the physical interaction between the environment and the lander

### Test 1: an arm fault happens during the GuardedMove operation.
The event trigger is the arm fault during the GuardedMove operation, as shown in Figure 1
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1_arm_fault_during_GuardedMove_operation.png) | 
|:--:| 
| *Figure 1: Adaptation trigger is the arm fault during the GuardedMove operation.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 2. Before re-planning, it will need to:
  * Wait for the arm fault to cause the GuardedMove operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1_adaptation_to_arm_fault_during_GuardedMove_operation.png) | 
|:--:| 
| *Figure 2: Adaptation is to remove the failed excavatino location and then re-planning* |

### Test 2: digging failure has happened once.
The event trigger is the arm fault during the Grind operation, as shown in Figure 3.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1_first_digging_failure.png) | 
|:--:| 
| *Figure 3: Adaptation trigger is the 1st digging failure.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 4. Before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1_adaptation_to_first_digging_failure.png) | 
|:--:| 
| *Figure 4: Adaptation is to remove the failed excavatino location and then re-planning* |

## Adaptation Case 2: The beliefs of machine learning models become lower than some thresholds.

### Test 1: digging failures have happened two times.
The event trigger is the arm fault during the Grind operation when the number of digging failures becomes 2, as shown in Figure 5.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2_second_digging_failure.png) | 
|:--:| 
| *Figure 5: Adaptation trigger is the 2nd digging failure.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 6. Before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations
  * Update the excavation-probability machine learning model
  * Update the excavation probabilities of the left candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2_adaptation_to_second_digging_failure.png)| 
|:--:| 
| *Figure 6: Adaptation is to update the excavation-probability model and the excavation probabilities of the left excavation location.* |

The newly synthesized plan results into a failure due to an unexpected GuardedMove failure. The failure comes with a warning, "Fail: ABORTED: no motion plan found. No execution attempted". More investigation regarding this failure should be done in order to prepare a proper adaptation.
Figure 7 Unexpected failure during the GuardedMove operation.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/Unknown_Failure_of_GuardedMove.png) | 
|:--:| 
| *Figure 7: The 5th excavation plan fails due to an unexpected failure of GuardedMove.* |
