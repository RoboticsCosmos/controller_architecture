import rdflib
from namespaces import ERROR_SIGNAL
from utility import resolver

class ErrorSignalTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:
        functions = dict()
        data_structures = dict()
        state_machine = dict()

        return {
            "functions": functions,
            "state_machine": state_machine,
            "data_structures": data_structures
        }                