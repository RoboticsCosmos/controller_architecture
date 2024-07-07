import os
import json
import rdflib
from rdflib import URIRef
from utility import resolver, helper

from ir_gen.translators import (
    ScheduleTranslator,
    MonitorTranslator,
    DataTranslator,
)

from namespaces import (
    ERROR,
    FUNCTIONS,
    CONTROLLER,
    ALGORITHM,
    ABAG,
    PID_CONTROLLER,
    IMPEDANCE_CONTROLLER,
    MONITOR,
    PLAN,
)

# from rdflib.term import URIRef # which one to use?
# RDFLib’s class for representing IRIs/URIs is called “URIRef”
"""
Terms are the kinds of objects that can appear in a RDFLib’s graph’s triples. 
Those that are part of core RDF concepts are: IRIs, Blank Node and Literal, 
the latter consisting of a literal value and either a datatype or an RFC 3066 
language tag.
"""


def get_subject_when_object_is_in_list(g, predicate, object, type_of_subject):

    for s in g.subjects(predicate=predicate, object=object):
        if (s, rdflib.RDF.type, type_of_subject) in g:
            return s
    return None


def main():

    METAMODELS = rdflib.Namespace("https://controller.org/metamodels")

    current_dir = (
        os.path.dirname(os.path.abspath(__file__))
        if "__file__" in globals()
        else os.getcwd()
    )
    meta_models_path = os.path.join(current_dir, "../metamodels/")
    models_path = os.path.join(current_dir, "../controller-models/uc1/")

    url_map = {METAMODELS: meta_models_path}
    resolver.install(resolver.IriToFileResolver(url_map))

    # 'ConjunctiveGraph' can load multiple graphs with different graphs into a single graph.
    g = rdflib.ConjunctiveGraph()
    g.bind(
        "uuid", rdflib.Namespace("urn:uuid:")
    )  # bind prefix("uuid") to namespace("urn:uuid:")

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(
                models_path + file, format="json-ld"
            )  # creating a graph from a JSON-LD file

    # print(g.serialize(format="turtle"))  # serialize the graph to a string in the specified format
    # *** subject; predicate; object ***/

    print("--" * 20)

    data_structures_dict = dict()
    functions_dict = dict()
    controller_architecture_per_dimension_list = []

    # populate all data structures from the model
    for data_id in g.subjects(predicate=rdflib.RDF.type, object=ALGORITHM.Data):
        data_name = g.compute_qname(data_id)[-1]
        ir = DataTranslator().translate(g, data_id)
        data_structures_dict[data_name] = ir

    # g.objects(): returns a generator of objects for the given subject and predicate
    # g.subjects(): returns a generator of subjects for the given predicate and object
    # g.value(): returns the value of the first object for the given subject and predicate

    ## Generator ##
    # a generator is a type of iterable, like a list or a tuple. Unlike lists, however, generators don't store all their values in memory at once; they generate each value on the fly as you iterate over them. This makes them very memory-efficient for large datasets.
    # to get the values from a generator, you can either iterate over it, or convert it to a list and then access the values
    # Eg: list(motion_spec_subject_gen) or for motion_spec in motion_spec_subject_gen: print(motion_spec)
    # once it is consumed, it is exhausted and you need to create a new generator if you need to iterate over it again

    if True:
        # for multiple motion specifications in each dimension
        for plan in g.subjects(
            predicate=rdflib.RDF.type, object=PLAN.Plan, unique=True
        ):
            motion_spec_per_dimension = dict()
            motion_specification_list = []

            dimension_of_control = g.compute_qname(plan)[-1]

            motion_spec_per_dimension["dimension_of_control"] = dimension_of_control
            motion_spec_per_dimension["pre_condition_satisfied"] = (
                dimension_of_control + "_is_active"
            )
            motion_spec_per_dimension["motion_specification_states"] = []

            data_structures_dict[dimension_of_control + "_is_active"] = {
                "data_type": "bool",
                "initial_value": "false",
            }

            # for each motion specification in the dimension
            for motion_spec in g.objects(subject=plan, predicate=PLAN.set_of_states):

                motion_spec_state_dict = {
                    "pre_condition": {
                        "function_names": [],  # function_names is added here so that while generating code, it is known which function to call in the begining of while loop
                        "flag_names": [],
                    },
                    "post_condition": {
                        "function_names": [],
                        "flag_names": [],
                    },
                    "schedules": dict(),
                }

                """
                schedules_dict_example = {
                    "schedule_1": {
                        "trigger_chain": [],
                        "monitors": {
                            "function_names": [],
                            "event_data": [],
                        },
                    },
                }
                """

                pre_conditions = g.objects(
                    subject=motion_spec,
                    predicate=PLAN.pre_condition,
                )
                post_conditions = g.objects(
                    subject=motion_spec, predicate=PLAN.post_condition
                )
                per_conditions = g.objects(
                    subject=motion_spec, predicate=PLAN.per_condition
                )

                # iterate through pre and post conditions and get all monitoring functions and data structures
                for pre_condition in pre_conditions:
                    constraint_monitor_id_gen = g.subjects(
                        predicate=MONITOR.constraint_to_monitor, object=pre_condition
                    )

                    for constraint_monitor_id in constraint_monitor_id_gen:

                        constraint_monitor_types, _ = helper.get_from_container(
                            subject_node=constraint_monitor_id,
                            predicate_value=rdflib.RDF.type,
                            graph=g,
                            return_just_id_after_hash=True,
                        )

                        if "ConstraintToFlag" in constraint_monitor_types:
                            flag_set_by_monitor_id = g.value(
                                subject=constraint_monitor_id,
                                predicate=MONITOR.flag_set_by_monitor,
                            )
                            if flag_set_by_monitor_id is None:
                                print(
                                    f"Error: flag_set_by_monitor_id is None for constraint_monitor_id: {constraint_monitor_id}"
                                )

                            ir = MonitorTranslator().translate(
                                g,
                                constraint_to_monitor_id=pre_condition,
                                flag_or_event_id=flag_set_by_monitor_id,
                                is_flag=True,
                            )
                            functions_dict.update(ir["functions"])
                            data_structures_dict.update(ir["data_structures"])
                            motion_spec_state_dict["pre_condition"][
                                "function_names"
                            ].append(ir["function_name"])
                            motion_spec_state_dict["pre_condition"][
                                "flag_names"
                            ].append(ir["flag_name"])
                            break

                        else:
                            pass

                for post_condition in post_conditions:

                    constraint_monitor_id_gen = g.subjects(
                        predicate=MONITOR.constraint_to_monitor, object=post_condition
                    )

                    for constraint_monitor_id in constraint_monitor_id_gen:
                        constraint_monitor_types, _ = helper.get_from_container(
                            subject_node=constraint_monitor_id,
                            predicate_value=rdflib.RDF.type,
                            graph=g,
                            return_just_id_after_hash=True,
                        )

                        if "ConstraintToFlag" in constraint_monitor_types:
                            flag_set_by_monitor_id = g.value(
                                subject=constraint_monitor_id,
                                predicate=MONITOR.flag_set_by_monitor,
                            )
                            ir = MonitorTranslator().translate(
                                g,
                                constraint_to_monitor_id=post_condition,
                                flag_or_event_id=flag_set_by_monitor_id,
                                is_flag=True,
                            )
                            functions_dict.update(ir["functions"])
                            data_structures_dict.update(ir["data_structures"])
                            motion_spec_state_dict["post_condition"][
                                "function_names"
                            ].append(ir["function_name"])
                            motion_spec_state_dict["post_condition"][
                                "flag_names"
                            ].append(ir["flag_name"])
                            break

                        else:
                            pass

                schedule_node_ids = set()
                controller_call_node_ids = set()
                # for each per condition
                for per_condition in per_conditions:

                    node_generator_for_controller_call = g.subjects(
                        predicate=ALGORITHM.constraints_controlled, object=per_condition
                    )

                    # if generator is empty, then it doesn't enter the for loop
                    for node_id in node_generator_for_controller_call:

                        if node_id not in controller_call_node_ids:
                            constraint_controller = g.value(
                                subject=node_id,
                                predicate=ALGORITHM.constraint_controller,
                            )
                            schedule_of_controller = g.value(
                                predicate=ALGORITHM.associated_controller,
                                object=constraint_controller,
                            )

                            schedule_node_ids.add(schedule_of_controller)

                        controller_call_node_ids.add(node_id)

                print("[runner.py] schedule_node_ids: ", schedule_node_ids)
                ir = ScheduleTranslator().translate(
                    g,
                    schedule_node_ids,
                    motion_spec,
                )

                functions_dict.update(ir["functions"])
                data_structures_dict.update(ir["data_structures"])
                helper.deep_update_dictionary(
                    motion_spec_state_dict["schedules"], ir["schedules"]
                )
                motion_specification_list.append(motion_spec_state_dict)

            motion_spec_per_dimension["motion_specification_states"] = (
                motion_specification_list
            )
            controller_architecture_per_dimension_list.append(motion_spec_per_dimension)

        ir = {
            "functions": functions_dict,
            "data_structures": data_structures_dict,
            "controller_architecture": controller_architecture_per_dimension_list,
        }

        json_obj = json.dumps(ir, indent=4)

        # print(json_obj)

        # save in a json file
        with open(
            "/home/melody-u18/Desktop/Thesis/controller_architecture/gen/irs/uc1_latest_30_June.json",
            "w",
        ) as f:
            f.write(json_obj)


if __name__ == "__main__":
    main()
