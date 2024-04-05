import rdflib
from namespaces import PID_CONTROLLER
from utility import resolver

class PIDControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        functions = dict()
        data_structures = dict()
        for a in g.subjects(rdflib.RDF.type, PID_CONTROLLER.PIDController):
            controller = a.split("#")[-1]
            prefix = controller.split("_")[0]+controller[-1]+"_"
            # print(controller)
            functions[controller] = dict()
            functions[controller]["name"] = controller
            functions[controller]["p_gain"] = prefix+"kp"
            functions[controller]["i_gain"] = prefix+"ki"
            functions[controller]["d_gain"] = prefix+"kd"
            functions[controller]["setpoint_pos"] = prefix+"setpoint_pos"
            functions[controller]["measured_pos"] = prefix+"measured_pos"
            functions[controller]["prev_error"] = prefix+"prev_error"
            functions[controller]["pos_error"] = prefix+"pos_error"
            functions[controller]["integral_term"] = prefix+"integral_term"
            functions[controller]["dt"] = prefix+"dt"
            functions[controller]["output"] = prefix+"control_command"

            # looping through the values of keys of functions[controller]

            for key in functions[controller]:
                data_structures[functions[controller][key]] = dict()
                # if key == "name":
                #     data_structures[functions[controller][key]] = dict()

            for key in functions[controller]:
                # print(key)
                # if key != "name":
                #     data_structures[controller][functions[controller][key]]["name"] = functions[controller][key]
                #     data_structures[controller][functions[controller][key]]["d_type"] = "float"
                #     data_structures[controller][functions[controller][key]]["init"] = 0.0
                #     data_structures[controller][functions[controller][key]]["is_pointer"] = False

        return {
            "functions": functions,
            "data_structures": data_structures
        }