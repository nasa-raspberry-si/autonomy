# Demo for the adaptation case 1 and case 2
After following the [steps](https://github.com/nasa-raspberry-si/autonomy/tree/ow8-rosnodes#run-the-raspberry-si-autonomy) to run the [mission1.txt](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/evaluation/mission1.txt), we trigger an adaptation during different lander arm operations by setting the arm fault parameter, **shou\_yaw\_effort\_failure**, in [the fault configuration tab](https://github.com/nasa/ow_simulator/wiki/Fault-Injection-and-Modeling/2f5ede92df2704a68d7a1b0436e2dc81612a29c5) in rqt (developed by [OceanWATERS](https://github.com/nasa/ow_simulator)). In this demo, we differentiate an arm fault into two groups based on when it occurs. In the first group, the arm fault happens during an Grind operation and causes it to be aborted. We call it a digging failure. In the second group, the arm fault happens during arm operations other than the Grind operation.

All tests below were conducted in one run. The recording of the experiment can be found [here](https://pjamshid-nas.us6.quickconnect.to/vs/sharing/nT3s55MM#!bW92aWUtMjU=).

## Adaptation Case 1: Failure caused by the physical interaction between the environment and the lander

### Test 1: an arm fault happens during the GuardedMove operation.
The event trigger is the arm fault during the GuardedMove operation, as shown in Figure 1
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case1_arm_fault_during_GuardedMove_operation.png) | 
|:--:| 
| *Figure 1: Adaptation trigger is the arm fault during the GuardedMove operation.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 2. Before re-planning, it will need to:
  * Wait for the arm fault to cause the GuardedMove operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case1_arm_fault_during_GuardedMove_operation_adaptation.png) | 
|:--:| 
| *Figure 2: Adaptation is to remove the failed excavatino location and then re-planning* |

### Test 2: digging failure has happened once.
The event trigger is the arm fault during the Grind operation, as shown in Figure 3.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case1_arm_fault_during_Grind_operation_1st_time.png) | 
|:--:| 
| *Figure 3: Adaptation trigger is the 1st digging failure.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 4. Before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case1_arm_fault_during_Grind_operation_1st_time_adaptation.png) | 
|:--:| 
| *Figure 4: Adaptation is to remove the failed excavatino location and then re-planning* |

## Adaptation Case 2: The beliefs of machine learning models become lower than some thresholds.

### Test 1: digging failures have happened two times.
The event trigger is the arm fault during the Grind operation when the number of digging failures becomes 2, as shown in Figure 5.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case2_arm_fault_during_Grind_operation_2nd_time.png) | 
|:--:| 
| *Figure 5: Adaptation trigger is the 2nd digging failure.* |

The adaptation is to synthesize a new excavation plan, as shown in Figure 6. Before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Remove the currently chosen excavation location from the list of candidate excavation locations
  * Update the excavation-probability machine learning model
  * Update the excavation probabilities of the left candidate excavation locations

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case2_arm_fault_during_Grind_operation_2nd_time_adaptation.png)| 
|:--:| 
| *Figure 6: Adaptation is to update the excavation-probability model and the excavation probabilities of the left excavation location.* |


### Test 2: digging failures have happened three times
The event trigger is the arm fault during the Grind operation while the number of digging failures becomes 3, as shown in Figure 7.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case2_arm_fault_during_Grind_operation_3rd_time.png) | 
|:--:| 
| *Figure 7: Adaptation trigger is the 3rd digging failure.* |

The adaptation is to obtain new lists of excavation and dump locations and then synthesize a new excavation plan, as shown in Figure 8. Before re-planning, it will need to:
  * Wait for the arm fault to cause the Grind operation to abort
  * Clear the arm fault and terminate the plan when necessary
  * Update the excavation-probability and science-value models
  * Randomly select the numbers of excavation locations and dump locations (This is a workaround for now. They should be determined by machine learning models later.)
  * Generate the list of excavation locations and the list of dump location

| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case2_arm_fault_during_Grind_operation_3rd_time.png) | 
|:--:| 
| *Figure 8: Adaptation is to update all machine learning models, re-generate the lists of excavation and dump locations, and re-plan.* |

Figure 9 shows the successful execution of the final excavation plan after the adaptation in test 2 of case 2.
| ![](https://github.com/nasa-raspberry-si/autonomy/blob/ow8-rosnodes/demo/case2_arm_fault_during_Grind_operation_3rd_time_adaptation_plan_succeeded.png) | 
|:--:| 
| *Figure 9: The 5th excavation plan is successfully executed.* |
