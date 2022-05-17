import os
from rospy import loginfo
import json
from math import sqrt

# A scenario represents a task in Europa Lander Mission
class Scenario:
    EXCAVATION = 1
    UNKNOWN = 2
   
    num_str_map = {
            1: "Excavation",
            2: "Unknown"}

    @classmethod
    def get_scenario_str(cls, scenario_num):
        # ToDo
        assert scenario_num <= cls.UNKNOWN
        return cls.num_str_map[scenario_num]

class CheckPoint:
    def __init__(self, cp_type, cp_name, cp_status):
        self.type = cp_type
        self.name = cp_name
        self.status = cp_status

class PlexilPlanTranslator():
    def __init__(self):

        self.scenarios_map = {
                "Excavation": Scenario.EXCAVATION,
                "Unknown": Scenario.UNKNOWN
                }

    # prism_syn_plan: [action1, action2, ...]
    # plexil_plan_name is the name of the plexil node.
    def translate(self, scenario_ID, runtime_info, plexil_plan_name, plexil_plan_dir, prism_syn_plan):
        scenario = self.scenarios_map[scenario_ID]
        # Parse prism_syn_plan
        loginfo("Parsing the synthesized plan by PRISM...")
        plan_info = self.parse_syn_plan(scenario, runtime_info, prism_syn_plan)
        plan_info["node_name"] = plexil_plan_name

        # Generate the PLEXIL code
        loginfo("Generating the PLEXIL code...")
        code = self.generate_code(scenario, plan_info)

        plexil_plan_fp = os.path.join(plexil_plan_dir, plexil_plan_name+".plp") 
        with open(plexil_plan_fp, "w") as outfile:
            outfile.write(code)

    # [Section: Parse Synthesized Plan By PRISM Model]
    def parse_syn_plan(self, scenario, runtime_info, prism_syn_plan):
        plan_info = {}
        if scenario == Scenario.EXCAVATION:
            plan_info = self.parse_syn_plan_exca(runtime_info, prism_syn_plan)
            plan_info['task_name'] = Scenario.EXCAVATION
        else:
            loginfo("[Plan Translation] Unknown scenario: " + Scenario.get_scenario_str(scenario))
        return plan_info

    ## Excavation
    def parse_syn_plan_exca(self, runtime_info, prism_syn_plan):
        plan_info = {'sel_xloc': {}, 'sel_dloc': {}}

        syn_plan = prism_syn_plan[1:-1].split(', ')
        # Currently only support the case of max_tried=1,
        # which assumes that each action starts with "select_"
        sel_xloc_ID = syn_plan[0][7:]
        sel_dloc_ID = syn_plan[1][7:]
        sel_xloc = runtime_info['xloc_list'][sel_xloc_ID]
        sel_dloc = runtime_info['dloc_list'][sel_dloc_ID]
        loginfo("[Synthesized Excavation Plan]:")
        
        loginfo(">>> selected excavation location: " + sel_xloc["name"] + ":")
        loginfo(json.dumps(sel_xloc, indent=4, sort_keys=True))
        loginfo(">>> selected dump location: " + sel_dloc["name"] + ":")
        loginfo(json.dumps(sel_dloc, indent=4, sort_keys=True))
        
        plan_info['sel_xloc'] = sel_xloc
        plan_info['sel_dloc'] = sel_dloc
        return plan_info


    # [Section: PLEXIL Code Generation]
    def generate_code(self, scenario, plan_info):
        code = ""
        if scenario == Scenario.EXCAVATION:
            code = self.gen_exca_scenario(plan_info)
        else:
            loginfo("[Plan Translation] Unknown scenario: " + scenario_ID)
        return code

    ## Excavation
    def gen_exca_scenario(self, plan_info):
        # [Information extracted from plan_info]
        # node_name is the name of the root node.
        node_name = plan_info["node_name"]
        sel_xloc = plan_info["sel_xloc"]
        sel_dloc = plan_info["sel_dloc"]
        xloc_x = sel_xloc['position']['x']
        xloc_y = sel_xloc['position']['y']
        xloc_sv = sel_xloc['sci_val']
        xloc_ep = sel_xloc['ex_prob']
        dloc_x = sel_dloc['position']['x']
        dloc_y = sel_dloc['position']['y']

        dloc_z = 0.02
        trench_depth=0.1 # meters
        trench_length=0.3 # meters


        # [PLEXIL plan]
        loginfo("Generating the PLEXIL code for excavation scenario...")

        # variable declarations inside the PLEXIL plan
        # Each declaration has the format:
        #   var_name: (var_type, var_default_value)
        #   A value of "" for var_default_value means no explicit default value
        var_decls = {
            "GroundDetectionSuccess": ("Boolean", "false"),
            "DiggingSuccess"        : ("Boolean", "false"),
            "TrenchReady"           : ("Boolean", "false"),

            "OpOutcome"             : ("String", ""),
            "PlanStatus"            : ("String", "")
        }

        plan_conditions = {
            "ExitCondition" : "Lookup(TerminatePlan)"
        }

        # "PlanStatus" and "OpOutcome" in the checkpoints dictionary correspond
        # to the declared variables in var_decls above in the create PLEXIL plan
        checkpoints = {
            # For Plan Status
            "Plan_Start" : CheckPoint("Plan", node_name+".plx", "PLAN_STARTED"),
            "Plan_Finish" : CheckPoint("Plan", node_name+".plx", "PlanStatus"),

            # For Operation Status
            "GroundDetection_Start": CheckPoint("Operation", "GroundDetection", "OP_STARTED"),
            "GroundDetection_Finish": CheckPoint("Operation", "GroundDetection", "OpOutcome"),
            "Digging_Start": CheckPoint("Operation", "Digging", "OP_STARTED"),
            "Digging_Finish": CheckPoint("Operation", "Digging", "OpOutcome"),
            "TailingRemoval_Start": CheckPoint("Operation", "TailingRemoval", "OP_STARTED"),
            "TailingRemoval_Finish": CheckPoint("Operation", "TailingRemoval", "OpOutcome"),
        }

        code = ""
        code += self.gen_plan_desc()
        code += self.gen_include_files(Scenario.EXCAVATION)
        code += node_name + ":\n"
        code += "{\n"

        # Variable declarations, Plan conditions and Unstow action
        code += self.gen_var_decls(1, var_decls)
        code += self.gen_plan_conditions(1, plan_conditions)
        code += self.gen_checkpoint(1, checkpoints["Plan_Start"])
        code += self.gen_log_info(1, "\"["+node_name+".plx] Start"+"\"")
        code += "\n"
        code += self.gen_plan_info(1, plan_info)
        code += self.gen_unstow(1)

        # GroundDetection node
        cp_start = checkpoints["GroundDetection_Start"]
        gd_node_body = self.gen_ground_detection_body(2, cp_start, xloc_x, xloc_y)
        code += self.gen_plexil_node(1, "GroundDetection", gd_node_body)

        # Check the result of GroundDetection
        ## gdo : ground detection outcome
        gdo_if_cond = "Lookup(GroundFound) && GroundDetection.outcome == SUCCESS"
        temp_var_assignments = {
            "GroundDetectionSuccess" : "true",
            "OpOutcome"              : "OP_SUCCESS"
        }
        gdo_if_body = self.gen_var_assignment(2, temp_var_assignments)
        gdo_if_body += self.gen_log_info(2, "\"Detected ground position: \", Lookup(GroundPosition)") 
        gdo_else_body = self.gen_var_assignment(2, {"OpOutcome" : "OP_FAILURE"}) 
        code += self.gen_if_stat(1, gdo_if_cond, gdo_if_body, gdo_else_body, [])
        ## Send out the result
        code += self.gen_checkpoint(1, checkpoints["GroundDetection_Finish"])
        code += self.gen_log_info(1, "\"[GroundDetection Operation] \", OpOutcome")
        code += "\n"

        # Continue the plan if GroundDetection is successful
        # Start generating the statement "if (GroundDetectionSuccess)"
        # gds : ground detection sucess
        gds_if_cond = "GroundDetectionSuccess"
        gds_if_body = ""
        
        ## Digging node
        ## dg : digging
        cp_start = checkpoints["Digging_Start"]
        dg_node_body = self.gen_digging_body(3, cp_start, xloc_x, xloc_y, trench_depth, trench_length)    
        dg_node = self.gen_plexil_node(2, "Digging", dg_node_body)
        gds_if_body += dg_node # add Digging node to the body of "if (GroundDetectionSuccess)"
        ## Check the result of Digging
        ### dgo : digging outcome
        dgo_if_cond = "Digging.outcome == SUCCESS"
        temp_var_assignments = {
            "DiggingSuccess" : "true",
            "OpOutcome"      : "OP_SUCCESS"
        }
        dgo_if_body = self.gen_var_assignment(3, temp_var_assignments)
        dgo_else_body = self.gen_var_assignment(3, {"OpOutcome" : "OP_FAILURE"}) 
        dgo_if_stat = self.gen_if_stat(2, dgo_if_cond, dgo_if_body, dgo_else_body, [])
        # add "if (Digging.outcome == SUCCESS) to the body of "if (GroundDetectionSuccess)"
        gds_if_body += dgo_if_stat
        ## Send out the result
        gds_if_body += self.gen_checkpoint(2, checkpoints["Digging_Finish"])
        gds_if_body += self.gen_log_info(2, "\"[Digging Operation] \", OpOutcome")
        gds_if_body += "\n"

        ## Continue the plan if Digging is successful
        ## Start generating the statement "if (DiggingSuccess)"
        ## dgs : digging success
        dgs_if_cond = "DiggingSuccess"
        dgs_if_body = ""

        ### TailingRemoval node
        cp_start = checkpoints["TailingRemoval_Start"]
        tr_node_body = self.gen_tailing_remove_body(4, cp_start, xloc_x, xloc_y, trench_depth, dloc_x, dloc_y, dloc_z)
        tr_node = self.gen_plexil_node(3, "TailingRemoval", tr_node_body)
        dgs_if_body += tr_node
        ### Check the result of TailingRemove
        #### tro : Trailing Removal Outcome
        tro_if_cond = "TailingRemoval.outcome == SUCCESS"
        temp_var_assignments = {
            "TrenchReady" : "true",
            "OpOutcome"   : "OP_SUCCESS"
        }
        tro_if_body = self.gen_var_assignment(4, temp_var_assignments)
        tro_else_body = self.gen_var_assignment(4, {"OpOutcome" : "OP_FAILURE"}) 
        tro_if_stat = self.gen_if_stat(3, tro_if_cond, tro_if_body, tro_else_body, [])
        dgs_if_body += tro_if_stat
        ### Send out the result
        dgs_if_body += self.gen_checkpoint(3, checkpoints["TailingRemoval_Finish"])
        dgs_if_body += self.gen_log_info(3, "\"[TailingRemoval Operation] \", OpOutcome")
        dgs_if_body += "\n"

        ## Close IF statement, "if (DiggingSuccess)"
        dgs_if_stat = self.gen_if_stat(2, dgs_if_cond, dgs_if_body)

        # Close IF statement for checking if ground detection succeeds
        gds_if_body += dgs_if_stat
        code += self.gen_if_stat(1, gds_if_cond, gds_if_body)

        # Check if the trench is ready
        code += self.gen_plan_finish_status(1, "TrenchReady")

        # Send out the status of the plan
        code += self.gen_checkpoint(tab_num=1, checkpoint=checkpoints["Plan_Finish"])
        code += self.gen_log_info(tab_num=1, str_expr="\"[" + node_name + ".plx] Finish\"")
        
        code += "}\n"
        return code
 
      
    # [Section: Plan Information]
    def gen_plan_desc(self):
        code = "// The following PLEXIL codes are automatically generated during planning.\n\n"
        return code
       
    def gen_plan_info(self, tab_num, plan_info):
        pre_dents = self.__gen_ident_str(tab_num)
        plan_info_str = json.dumps(plan_info)
        # " => \"
        plan_info_str = plan_info_str.replace("\"", "\\\"")
        code = pre_dents + "log_info (\"" + plan_info_str + "\");\n\n"
        return code

    # [Section: PLEXIL Langauge Code]
    def gen_include_files(self, scenario):
        code = "#include \"plan-interface.h\"\n"
        # FIXME: here we can include additonal stuff for different tasks
        #        or, include different header when testing against OWLAT
        if scenario == Scenario.EXCAVATION:
            pass
        code += "\n\n"
        return code

    def gen_log_info(self, tab_num, str_expr):
        pre_dents = self.__gen_ident_str(tab_num)
        code = "log_info (" + str_expr + ");\n"
        code = [code]
        return self.__ident_code(code, pre_dents)

    def gen_var_decls(self, tab_num, var_decls):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        for var_name, var_type_default in var_decls.items():
            var_decal = var_type_default[0] + " " + var_name
            if var_type_default[1]!="": # if there is a default value
                var_decal += "=" + var_type_default[1]
            code.append(var_decal + ";\n")
        code.append("\n")
        return self.__ident_code(code, pre_dents)

    def gen_var_assignment(self, tab_num, var_assignments):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        for var_name, var_expr in var_assignments.items():
            code.append(var_name + "=" + var_expr + ";\n")
        return self.__ident_code(code, pre_dents)

    def gen_plan_conditions(self, tab_num, plan_conditions):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        for cond_name, cond_expr in plan_conditions.items():
            code.append(cond_name + " " + cond_expr + ";\n")
        code.append("\n")
        return self.__ident_code(code, pre_dents)

    # Support checkpoint that is enabled via PLEXIL Update node
    # to send information from the executing PLEXIL plan to the autonomy
    # The input parameter, checkpoint, is an instance of CheckPoint
    def gen_checkpoint(self, tab_num, checkpoint):
        pre_dents = self.__gen_ident_str(tab_num)
        code = " ".join((
            "Update",
            "checkpoint_type=\""   + checkpoint.type + "\",",
            "checkpoint_name=\""   + checkpoint.name + "\",",
            "checkpoint_status=" + checkpoint.status + ";\n"))
        code = [code]
        return self.__ident_code(code, pre_dents)
    
    # The parameter, "tab_num" applies to if/elsif condition line and else
    # For branch bodies, they are strings and are assumed to include the necessnary number of tabs
    # For each item in the elseif_branches:
    #   the 1st element is the condition
    #   the 2nd element is the branch body
    def gen_if_stat(self, tab_num, cond, body, else_body="", elseif_branches=[]):
        pre_dents = self.__gen_ident_str(tab_num)
        # if branch
        code = pre_dents + "if (" + cond + ") {\n"
        code += body
        code += pre_dents + "}\n"
        # elseif branches
        for el_branch in elseif_branches:
            code += pre_dents + "elseif (" + el_branch[0] +") {\n"
            code += el_branch[1]
            code += pre_dents + "}\n"
        # else branch
        if else_body != "":
            code += pre_dents + "else {\n"
            code += else_body
            code += pre_dents + "}\n"
        code += "\n"

        return code


    # [Section: Lander Operation Code]
    def gen_unstow(self, tab_num):
        pre_dents = self.__gen_ident_str(tab_num)
        code = ["log_info (\"[Unstow] Start\");\n"]
        code.append("LibraryCall Unstow();\n")
        code.append("log_info (\"[Unstow] Finish\");\n\n")
        return self.__ident_code(code, pre_dents)

    def gen_stow(self, tab_num):
        pre_dents = self.__gen_ident_str(tab_num)
        code = ["log_info (\"Stowing arm...\");\n"]
        code.append("LibraryCall Stow();\n\n")
        return self.__ident_code(code, pre_dents)

    def gen_guarded_move(self, tab_num, x, y, z, dir_x, dir_y, dir_z, search_dist, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Start GuardedMove...\");\n")

        code.append("LibraryCall GuardedMove (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z) + ",\n")
        code.append("\tDirX = " + str(dir_x) + ", DirY = " + str(dir_y) + ", DirZ = " + str(dir_z) + ",\n")
        code.append("\tSearchDistance = " + str(search_dist) + ");\n")

        return self.__ident_code(code, pre_dents)

    def gen_grind(self, tab_num, x, y, depth=0.05, length=0.3, parallel=True, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Start Grind...\");\n")

        code.append("LibraryCall Grind (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ",\n")
        code.append("\tDepth = " + str(depth) + ", Length = " + str(length) + ",\n")
        code.append("\tGroundPos = Lookup(GroundPosition),\n")
        code.append("\tParallel = " + self.__get_bool_str(parallel) + ");\n")
 
        return self.__ident_code(code, pre_dents)


    def gen_dig_circular(self, tab_num, x, y, depth=0.05, parallel=True, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Start dig_circular...\");\n")
        code.append("LibraryCall DigCircular (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ", Depth = " + str(depth) + ",\n")
        code.append("\tGroundPos = Lookup(GroundPosition),\n")
        code.append("\tParallel = " + self.__get_bool_str(parallel) + ");\n")
 
        return self.__ident_code(code, pre_dents)

    def gen_deliver(self, tab_num, x, y, z=0.5,  msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Delivering the sample...\");\n")
        code.append("LibraryCall Deliver (X = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z) + ");\n")
        
        return self.__ident_code(code, pre_dents)

    # [Section: Utility for Operation PLEXIL Node]
    # gen_plexil_node(): generate the frame of a node
    # gen_XXX_body(): generate the node body that is used by gen_plexil_node()
    #                 it includes PLEXIL Update node and additional log_info
    #                 statement beside the corresponding code of the lander
    #                 operation.
    def gen_plexil_node(self, tab_num, node_name, node_body):
        pre_dents = self.__gen_ident_str(tab_num)
        # the head of the node
        code = pre_dents + node_name + ": {\n"
        code += node_body
        code += pre_dents + "}\n\n"
        return code


    def gen_ground_detection_body(self, tab_num, cp_start, x, y, z=0.05, dir_x=0.0, dir_y=0.0, dir_z=1.0, search_dist=0.5):
        code = ""
        ## Send out the message that GroundDetection starts
        code += self.gen_checkpoint(tab_num, cp_start)
        temp_msg = "[GroundDetection Operation] Start: a guarded move to find out the ground postion of the location (x=" + str(x) + ", y=" + str(y) + ")"
        code += self.gen_log_info(tab_num, str_expr="\"" + temp_msg + "\"")
        code += "\n"

        code += self.gen_guarded_move(tab_num, x, y, z, dir_x, dir_y, dir_z, search_dist)
        return code

    def gen_digging_body(self, tab_num, cp_start, x, y, depth=0.1, length=0.3, parallel=True):
        code = ""
        ## Send out the message that GroundDetection starts
        code += self.gen_checkpoint(tab_num, cp_start)
        temp_msg = "[Digging Operation] Start: grinding at the location (x=" + str(x) + ",y=" + str(y) + ")"
        code += self.gen_log_info(tab_num, "\"" + temp_msg + "\"")
        code += "\n"

        code += self.gen_grind(tab_num, x, y, depth, length, parallel)
        return code

    def gen_tailing_remove_body(self, tab_num, cp_start, xloc_x, xloc_y, trench_depth, dloc_x, dloc_y, dloc_z):
        code = ""
        # Send out the message that GroundDetection starts
        code += self.gen_checkpoint(tab_num, cp_start)
        temp_msg = "[TailingRemoval Operation] Start: remove from the depth of " + str(trench_depth) + " meters in the trench location (x=" + str(xloc_x) + ",y=" + str(xloc_y) + ") to the dump location (x=" + str(dloc_x) + ",y=" + str(dloc_y) + ",z=" + str(dloc_x) + ")"
        code += self.gen_log_info(tab_num, "\"" + temp_msg + "\"")
        code += "\n"

        # Grab tailing using DigCircular
        code += self.gen_dig_circular(tab_num, xloc_x, xloc_y, trench_depth)

        # Move tailing to the dump location
        code += self.gen_deliver(tab_num, dloc_x, dloc_y, dloc_z)

        return code

    # finish_status is an expression that can be used to access
    # to access to the plan status after evaluating it.
    def gen_plan_finish_status(self, tab_num, finish_status):
        if_cond = finish_status
        if_body = self.gen_var_assignment(tab_num+1, {"PlanStatus" : "PLAN_SUCCESS"})
        else_body = self.gen_var_assignment(tab_num+1, {"PlanStatus" : "PLAN_FAILURE"})
        return self.gen_if_stat(tab_num, if_cond, if_body, else_body)



    # [Section: utility functions]
    #
    # Python and Plexil have different boolean literals
    # Language   Boolean Literals
    # Python     True   |   False
    # Plexil     true   |   false
    def __get_bool_str(self, bool_var):
        if bool_var:
            return "true"
        else:
            return "false"

    def __ident_code(self, code, pre_dents):
        code_str = ""
        for line in code:
            code_str += pre_dents+line
        return code_str

    def __gen_ident_str(self, tab_num):
        return "\t"*tab_num
