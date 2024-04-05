import rdflib
from namespaces import IMPEDANCE_CONTROLLER
from utility import resolver

class ImpedanceControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:
        functions = dict()
        data_structures = dict()
        state_machine = dict()

        return {
            "functions": functions,
            "state_machine": state_machine,
            "data_structures": data_structures
        }                