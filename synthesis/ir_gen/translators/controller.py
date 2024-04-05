import rdflib
from namespaces import CONTROLLER
from utility import resolver

class ControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        functions = dict()
        data_structures = dict()
        for a in g.subjects(rdflib.RDF.type, CONTROLLER.Controller):
            controller = a.split("#")[-1]
            print(controller)
            functions[controller] = dict()
            functions[controller]["name"] = controller
            functions[controller]["error"] = "error_term"
            functions[controller]["gain"] = "gain"
            functions[controller]["output"] = "output"
            functions[controller]["param_pefix"] = controller.split("_")[0]

            data_structures[controller] = dict()
            # looping through the values of keys of functions[controller]

            for key in functions[controller]:
                if key == "name":
                    data_structures[controller][functions[controller][key]] = dict()

            for key in functions[controller]:
                print(key)
                # if key != "name":
                #     data_structures[controller][functions[controller][key]]["name"] = functions[controller][key]
                #     data_structures[controller][functions[controller][key]]["d_type"] = "float"
                #     data_structures[controller][functions[controller][key]]["init"] = 0.0
                #     data_structures[controller][functions[controller][key]]["is_pointer"] = False

        return {
            "functions": functions,
            "data_structures": data_structures
        }