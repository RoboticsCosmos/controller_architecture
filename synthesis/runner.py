import os
import json
import rdflib
from rdflib import URIRef
from ir_gen.translators import (
    PIDControllerTranslator,
    ImpedanceControllerTranslator,
    FunctionTranslator,
    ErrorSignalTranslator,
    ControllerTranslator,
    PlanTranslator,
    MonitorTranslator
)
from utility import resolver
from namespaces import (
    PID_CONTROLLER,
    IMPEDANCE_CONTROLLER,
    FUNCTIONS,
    ERROR_SIGNAL,
    CONTROLLER,
    PLAN,
    MONITOR,
)

# from rdflib.term import URIRef # which one to use?
# RDFLib’s class for representing IRIs/URIs is called “URIRef”
"""
Terms are the kinds of objects that can appear in a RDFLib’s graph’s triples. 
Those that are part of core RDF concepts are: IRIs, Blank Node and Literal, 
the latter consisting of a literal value and either a datatype or an RFC 3066 
language tag.
"""


def main():

    METAMODELS = rdflib.Namespace("https://controller.org/metamodels")

    meta_models_path = os.path.join(os.path.dirname(__file__), "../metamodels/")
    models_path = os.path.join(os.path.dirname(__file__), "../controller-models/")

    url_map = {METAMODELS: meta_models_path}
    resolver.install(resolver.IriToFileResolver(url_map))

    # 'ConjunctiveGraph' can load multiple graphs with different graphs into a single graph.
    g = rdflib.ConjunctiveGraph()

    g.bind(
        "uuid", rdflib.Namespace("urn:uuid:")
    )  # bind prefix("uuid") to namespace("urn:uuid:")

    # g.bind("controller_model", rdflib.Namespace("http://controller.org/")) # bind prefix("controller") to namespace("http://controller.org/")
    # g.bind("pid_controller_metamodel", rdflib.Namespace("https://controller.org/metamodels/PID/"))

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(
                models_path + file, format="json-ld"
            )  # creating a graph from a JSON-LD file

    print(
        g.serialize(format="turtle")
    )  # serialize the graph to a string in the specified format
    # subject; predicate; object

    # p_controller_model_IRI = URIRef("http://controller.org/PID/#myPController")
    # pid_controller = rdflib.Namespace("https://controller.org/metamodels/PID/")
    # controller_dot_org_NS = rdflib.Namespace("http://controller.org/")

    # if (p_controller_model_IRI, None, None) in g:
    #     print(">>>>>>>>>This graph contains triples about p_controller!\n")

    print("--" * 20)

    s = g.value(None, rdflib.RDF.type, PID_CONTROLLER.PIDController)

    functions = dict()
    data_structures = dict()
    state_machine = dict()

    # # functions[controller] = PIDControllerTranslator().translate(g, s)

    # # json_dict = {
    # #     "constant": g.value(s, p_controller_NS.constant, None),
    # #     "error": g.value(s, p_controller_NS.error, None),
    # #     "gain": g.value(s, p_controller_NS.gain, None),
    # #     "output": g.value(s, p_controller_NS.output, None)
    # # }

    ir = PIDControllerTranslator().translate(g, s)

    json_obj = json.dumps(ir, indent=4)

    print(json_obj)


if __name__ == "__main__":
    main()


"""
@prefix controller: <http://controller.org/> .
@prefix p_controller: <http://controller.org/PID/PController/> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .

<http://controller.org/PID/#myPController> a <http://controller.org/PID/PController> ;
    p_controller:output controller:output_definition ;
    controller:constant "0.5"^^xsd:float ;
    controller:error controller:error_definition ;
    controller:gain controller:kp .


----------------------------------------
>>>>>>>>>This graph contains triples about p_controller!

http://controller.org/PID/#myPController
http://controller.org/PID/#myPController
http://controller.org/PID/#myPController
http://controller.org/PID/#myPController
http://controller.org/PID/#myPController

"""
