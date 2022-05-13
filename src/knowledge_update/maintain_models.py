## 
# Current Model Design
# 1. Science value of a location is randomly drawn from (0, 1)
# 2. Excavation probability of a location is set inversely
#    propostional to the science value such that planning problem
#    could be sort of interesting when no real data is available.
#
# Model Files Parsing
# The first line of a model file represents the most recent model
##

from rospy import loginfo

from datetime import datetime
import random

class ModelUpdater(): 
    def __init__(self, models_dir, model_names, exca_prob_beta = 0.4):
        self.models_dir = models_dir
        self.model_names = model_names
        self.ExcaProb_beta = exca_prob_beta

    def writeToFile(self, filepath, content):
        loginfo("Updating model file: {}".format(filepath))
        f = open(filepath, "a")
        f.write(content)
        f.close()

    def updateScienceValueModel(self):
        # a string of datetime containing current date and time
        now_str = datetime.now().isoformat()
        content = now_str + " | Updated\n"
        self.writeToFile(fp, content)
        loginfo("Science-Value Model is updated now.")

    # randomly draw the beta value from [0.3, 0.5].
    # The range has no special meaning.
    def updateExcaProbModel(self):
        self.beta = 0.3 + 0.2 * random.random()

        # a string of datetime containing current date and time
        now_str = datetime.now().isoformat()
        content = now_str + " | beta:" + str(beta) + "\n"
        self.writeToFile(fp, content)
        loginfo("Excavation-Probability Model Updated: beta value now is {}".format(beta))

    def updateModel(self, model_name):
        success = True
        fp = self.models_dir + "/" + model_name + ".model"
        if model_name == "SciVal":
            self.updateScienceValueModel()
        elif model_name == "ExcaProb":
            self.updateExcaProbModel()
        else:
            loginfo("Unsupported model: {}".format(model_name))
            msg = "Currently supported models:"
            for model_name in self.model_names:
                msg = msg + "\n\t" + model_name
            loginfo(msg)
            success = False
        return success

    def updateModels(self, model_names):
        success = True
        results = []
        for model_name in model_names:
            result = self.updateModel(model_name)
            results.append(result)
        if False in results:
            sucess = False
        return success

