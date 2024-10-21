import rdflib
from namespaces import ALGORITHM
from utility import resolver

class DataTranslator:

    def translate(self, g: rdflib.Graph, data_id) -> dict:
        ir = dict()
        data_name = g.compute_qname(data_id)[-1]
        ir["data_type"] = g.value(subject=data_id, predicate=ALGORITHM.data_type)
        ir["initial_value"] = g.value(subject=data_id, predicate=ALGORITHM.init_value)
        return ir
