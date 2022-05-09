// autonomy-excavate
// RASPBERRY-SI Planning model prototype for Ocean Worlds Autonomy Testbed
// Javier Camara, Univerity of York
// javier.camaramoreno@york.ac.uk

mdp


#const NXLOCS# // Number of excavation locations
#const NDLOCS# // Number of dump locations

// Parameters
const curLoc=#NXLOCS+NDLOCS+1#; // Current location of arm when planner is called (here we have a hardcoded value, but this will be provided to the planner)

// Special Locations
const locOrig=0;
const locNull=#NXLOCS+NDLOCS#+100; // Only auxiliary constants to define the range of variable loc

// Excavation Locations
#for i=1:NXLOCS#
const xloc#i#=#i#;
#end#

// Dump Locations
#for i=1+NXLOCS:NXLOCS+NDLOCS#
const dloc#i-NXLOCS#=#i#;
#end#


// Possible states of the mission (to keep track of progress)
const START=0;
const DONE_EXCAVATING=1;
const DONE_DUMPING=2;
const FAILED=100;

formula tried_all_xloc = #for i=1:NXLOCS# tried_xloc#i# & #end# true; // Have we tried all excavation locations?
//const int MAX_TRIED; // Maximum number of excavation attempts (can be 1 by default, number of excavation locations upper bound)

// Module that is in charge of selecting an excavation location
// It tries one location first, and if it does not succeed, it goes to the next one.
// If all locations have been tried and state is still START (unsuccessful excavation), it fails
module autonomy
	s:[START..FAILED] init START; // State of mission
	loc:[locOrig..locNull] init curLoc;
	tried:[0..MAX_TRIED] init 0; // Keeps track of excavation attempts

	
	#for i=1:NXLOCS#
	tried_xloc#i#: bool init false;
	succ_xloc#i#: bool init false;
	#end#

	// Excavation behavior
	// In the following commands, we have:
	// Command 1:
	// * A guard that checks: (1) that we are at the START of the mission
	// 			  (2) that we have not tried to excavate location A
	//			  (3) that the maximum number of excavation attempts has not been reached
	// * An update with probability ex_locA (excavatability of excavation location A) that:
	//			  (1) updates the variable saying that we have tried location A (not needed but left for clarity)
	// 			  (2) updates the variable that keeps track of excavation attempts
	//			  (3) updates the arm location variable to location A
	//            (4) updates the variable for excavation success in the current location to true	
	// * Another update with probability 1-ex_locA that:
	//			  (1) updates the variable saying that we have tried location A
	//			  (2) updates the arm location variable to location A (even if we have not succeeded excavating)
	// 			  (3) updates the variable that keeps track of excavation attempts
	// Command 2:
	// * A guard that checks: (1) that there is success in the current excavation location
	// * An update that: (1) sets the excavation success back to false
   	//			         (2) updates the state to successful excavation (DONE_EXCAVATING)

	
	#for i=1:NXLOCS#
	[try_xloc#i#] (s=START) & (!tried_xloc#i#) & (tried<MAX_TRIED) -> ex_loc#i#: (s'=DONE_EXCAVATING) & (tried_xloc#i#'=true) &  (tried'=tried+1)  & (loc'=xloc#i#) & (succ_xloc#i#'=true) 
				  	       + (1-ex_loc#i#): (tried_xloc#i#'=true) &  (tried'=tried+1) & (loc'=xloc#i#);
	[select_xloc#i#] (succ_xloc#i#) -> (succ_xloc#i#'=false) & (s'=DONE_EXCAVATING);
	#end#

	
	// If all excavation locations have been tried (or maximum number of excavation attempts has been reached) and state is not DONE_EXCAVATING, mission fails
	[] (s=START) & (tried_all_xloc | tried>= MAX_TRIED) -> (s'=FAILED); 

	// Dump behavior
	// These commands just update the state to DONE_DUMPING (no probability of failure), and update arm location

	#for i=1:NDLOCS#
	[select_dloc#i#] (s=DONE_EXCAVATING) -> (s'=DONE_DUMPING) & (loc'=dloc#i#);
	#end#


endmodule


// stopping condition label for PCTL formula checking
label "done" = (s=DONE_DUMPING);


