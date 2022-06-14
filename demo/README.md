# Demo for the adaptation case 1 and case 2
After following the [steps](https://github.com/nasa-raspberry-si/autonomy/tree/ow9#run-the-raspberry-si-autonomy) to run the [mission1.txt](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/evaluation/mission1.txt), we trigger an adaptation during different lander arm operations by randomly setting an arm fault parameter (e.g., scoop_yaw_joint_locked_failure) in [the fault configuration tab](https://github.com/nasa/ow_simulator/wiki/Fault-Injection-and-Modeling/7d6438354885c32fea86fc294a62bef33eb0a18c) in rqt (developed by [OceanWATERS](https://github.com/nasa/ow_simulator)). In this demo, we differentiate an arm fault into two groups based on when it occurs. In the first group, the arm fault happens during arm operations other than the Grind operation. The adaptation strategy is to remove the currently selected excavation location and runs a re-planning with the updated list of excavation location. In the second group, the arm fault happens during an Grind operation and causes it to be aborted. We call it a digging failure. Currently, we use the number of digging failures (num_digging_failure) to trigger different adaptations:
   - num_digging_failure == 1: same adaptation strategy as that applied when seeing the arm fault in the first group.
   - num_digging_failure == 2: the lander loses the trust to the model that predict the successful excavation probability of an location. That is, the lander need to update the model and get the new excavation probability for each existing excavation location in the list, and then run a re-planning with the updated list.
   - num_digging_failure == 3: the lander loses the trust to all models and need to update them all. That is, the lander decides to discard the current lists of locations for the excavation task and updated models to generate new lists of candidate excavation and dump locations. Then, the lander issues a re-planning with the lists.

All tests below were conducted in one run where the initial excavation plan, ExcavationPlan1.plp, selects the excavation location (x=1.49, y=0.6, sci_val=0.9328, ex_prob=0.6893) from a list of 8 candidate locations and the dump location (x=1.52, y=-0.3) from a list of 4 candidate dump locations. Here, sci_val and ex_prob refer to the science value and the successful excavation probability of the excavation location. The recording of the experiment can be found [here](https://pjamshid-nas.us6.quickconnect.to/vs/sharing/nT3s55MM#!bW92aWUtMjc=) and the evaluation details can be found in [evaluation/Tasks/Excavation](https://github.com/nasa-raspberry-si/autonomy/tree/ow9/evaluation/Tasks/Excavation).

## Adaptation Case 1: Failure caused by the physical interaction between the environment and the lander

### Test 1: an arm fault happens during the GuardedMove operation.
The event trigger is the arm fault during the GuardedMove operation, as shown in Figure 1
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1test1_arm_fault_during_GuardedMove_operation.png) | 
|:--:| 
| *Figure 1: Adaptation trigger is the arm fault during the GuardedMove operation.* |

The adaptation is to synthesize a new excavation plan, ExcavationPlan2.plp. As shown in Figure 2, the new plan selects the excavation location (x=1.71, y=-0.4, sci_val=0.6761, ex_prob=0.7813) and the dump location (x=1.52, y=-0.3). And before re-planning, it will need to:
  * Wait for the arm fault to cause the GuardedMove operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1test1_adaptation_to_arm_fault_during_GuardedMove_operation.png) | 
|:--:| 
| *Figure 2: Adaptation is to remove the failed excavatino location and then re-planning* |

### Test 2: digging failure has happened once.
The event trigger is the arm fault during the Grind operation, as shown in Figure 3.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1test2_first_digging_failure.png) | 
|:--:| 
| *Figure 3: Adaptation trigger is the 1st digging failure.* |

The adaptation is to synthesize a new excavation plan, ExcavationPlan3.plp. As shown in Figure 4, the new plan selects the excavation location (x=1.54, y=-0.1, sci_val=0.8322, ex_prob=0.7308) and the dump location (x=1.52, y=-0.3). And before re-planning, it will need to performs the same operations as that in Test 1.

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case1test2_adaptation_to_first_digging_failure.png) | 
|:--:| 
| *Figure 4: Adaptation is to remove the failed excavatino location and then re-planning* |

## Adaptation Case 2: The beliefs of machine learning models become lower than some thresholds.

### Test 1: digging failures have happened two times.
The event trigger is the arm fault during the Grind operation when the number of digging failures becomes 2, as shown in Figure 5.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test1_second_digging_failure.png) | 
|:--:| 
| *Figure 5: Adaptation trigger is the 2nd digging failure.* |

The adaptation is to synthesize a new excavation plan, ExcavationPlan4.plp. As shown in Figure 6, the new plan selects the excavation location (x=1.48, y=-0.7, sci_val=0.1785, ex_prob=0.9329) and the dump location (x=1.52, y=-0.3). And before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations
  * Update the excavation-probability machine learning model
  * Update the excavation probabilities of the left candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test1_adaptation_to_second_digging_failure.png)| 
|:--:| 
| *Figure 6: Adaptation is to update the excavation-probability model and the excavation probabilities of the left excavation location.* |

### Test 2: digging failures have happened three times.
The event trigger is the arm fault during the Grind operation when the number of digging failures becomes 3, as shown in Figure 7.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test2_third_digging_failure.png) | 
|:--:| 
| *Figure 7: Adaptation trigger is the 3rd digging failure.* |

The adaptation is to synthesize a new excavation plan, ExcavationPlan5.plp. Figure 8 shows that 16 new excavation locations and 5 new dump locations are provided by the updated machine-learning models. Figure 9 shows that the new plan selects the excavation location (x=1.67, y=0.8, sci_val=0.3887, ex_prob=0.855) and the dump location (x=1.53, y=-0.2) for the new lists of locations. And before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Update the excavation-probability and science-value models
  * Randomly select the numbers of excavation locations and dump locations (This is a workaround for now. They should be determined by machine learning models later.)
  * Generate the list of excavation locations and the list of dump location

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test2_adaptation_to_third_digging_failure.png)| 
|:--:| 
| *Figure 8: Adaptation: update all machine learning models to re-generate the lists of excavation and dump locations.* |

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test2_adaptation_to_third_digging_failure.png)| 
|:--:| 
| *Figure 9: Adaptation: re-planning to synthesize a new excavation plan with the updated lists of locations.* |


Figure 10 shows the run-time adaptation helps the lander to successfully complete the excavation task in ExcavationPlan5.plp.


| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/demo/case2test2_third_digging_failure_new_plan_succeeded.png) | 
|:--:| 
| *Figure 10: The excavation task is successfuly completed in ExcavationPlan5.plp plan.* |
