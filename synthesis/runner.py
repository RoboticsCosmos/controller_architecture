import os
import json
import rdflib
from rdflib import URIRef
from utility import resolver, helper

from ir_gen.translators import (
    ScheduleTranslator,
    MonitorTranslator,
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


def main():

    METAMODELS = rdflib.Namespace("https://controller.org/metamodels")

    current_dir = (
        os.path.dirname(os.path.abspath(__file__))
        if "__file__" in globals()
        else os.getcwd()
    )
    meta_models_path = os.path.join(current_dir, "../metamodels/")
    models_path = os.path.join(current_dir, "../controller-models/uc1_z_axis_linear/")

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

    # print(g.serialize(format="turtle"))  # serialize the graph to a string in the specified format
    # *** subject; predicate; object ***/

    # p_controller_model_IRI = URIRef("http://controller.org/PID/#myPController")
    # pid_controller = rdflib.Namespace("https://controller.org/metamodels/PID/")
    # controller_dot_org_NS = rdflib.Namespace("http://controller.org/")

    # if (p_controller_model_IRI, None, None) in g:
    #     print(">>>>>>>>>This graph contains triples about p_controller!\n")

    print("--" * 20)

    behaviors_dict = dict()
    data_structures_dict = dict()
    functions_dict = dict()

    # g.objects(): returns a generator of objects for the given subject and predicate
    # g.subjects(): returns a generator of subjects for the given predicate and object
    # g.value(): returns the value of the first object for the given subject and predicate

    ## Generator ##
    # a generator is a type of iterable, like a list or a tuple. Unlike lists, however, generators don't store all their values in memory at once; they generate each value on the fly as you iterate over them. This makes them very memory-efficient for large datasets.
    # to get the values from a generator, you can either iterate over it, or convert it to a list and then access the values
    # Eg: list(behavior_subject_gen) or for behavior in behavior_subject_gen: print(behavior)
    # once it is consumed, it is exhausted and you need to create a new generator if you need to iterate over it again

    for behavior in g.subjects(
        predicate=rdflib.RDF.type, object=PLAN.Behavior, unique=True
    ):
        behavior_name_from_id = g.compute_qname(behavior)[-1]
        behaviors_dict[behavior_name_from_id] = {
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
                    "flag_names": [],
                },
            },
        }
        """

        pre_conditions = g.objects(subject=behavior, predicate=PLAN.pre_condition)
        post_conditions = g.objects(subject=behavior, predicate=PLAN.post_condition)
        per_conditions = g.objects(subject=behavior, predicate=PLAN.per_condition)
        arm_name = g.value(subject=behavior, predicate=PLAN.arm_name)
        direction_of_specification = g.value(
            subject=behavior, predicate=PLAN.direction_of_specification
        )

        # iterate through pre and post conditions and get all monitoring functions and data structures
        for pre_condition in pre_conditions:

            ir = MonitorTranslator().translate(
                g, constraint_to_monitor_id=pre_condition
            )
            functions_dict.update(ir["functions"])
            data_structures_dict.update(ir["data_structures"])
            behaviors_dict[behavior_name_from_id]["pre_condition"]["function_names"].append(ir["function_name"])
            behaviors_dict[behavior_name_from_id]["pre_condition"]["flag_names"].append(ir["flag_name"])

        for post_condition in post_conditions:

            ir = MonitorTranslator().translate(
                g, constraint_to_monitor_id=post_condition
            )
            functions_dict.update(ir["functions"])
            data_structures_dict.update(ir["data_structures"])
            behaviors_dict[behavior_name_from_id]["post_condition"]["function_names"].append(ir["function_name"])
            behaviors_dict[behavior_name_from_id]["post_condition"]["flag_names"].append(ir["flag_name"])

        schedule_call_node_ids = set()
        for per_condition in per_conditions:

            # assumption: schedule call can be purely monitor based(>=1), constraint based(=1), or both
            node_generator_for_constraint_based_schedule_call = g.subjects(
                predicate=PLAN.constraint_for_schedule, object=per_condition
            )
            node_generator_for_monitor_based_schedule_call = g.subjects(
                predicate=PLAN.monitors, object=per_condition
            )

            for node_id in node_generator_for_constraint_based_schedule_call:
                schedule_call_node_ids.add(node_id)
            for node_id in node_generator_for_monitor_based_schedule_call:
                schedule_call_node_ids.add(node_id)

        print("[runner.py] schedule_call_node_ids: ", schedule_call_node_ids)
        ir = ScheduleTranslator().translate(
            g, schedule_call_node_ids, behavior, arm_name, direction_of_specification
        )

        functions_dict.update(ir["functions"])
        data_structures_dict.update(ir["data_structures"])
        helper.deep_update_dictionary(
            behaviors_dict[behavior_name_from_id]["schedules"], ir["schedules"]
        )

    ir = {
        "functions": functions_dict,
        "data_structures": data_structures_dict,
        "behaviors": behaviors_dict,
    }

    json_obj = json.dumps(ir, indent=4)

    print(json_obj)

    # save in a json file
    with open("/home/melody-u18/Desktop/Thesis/controller_architecture/gen/irs/uc1_lin_z_axis_arm_1.json", "w") as f:
        f.write(json_obj)


if __name__ == "__main__":
    main()

    # controllers_to_activate_list, _ = get_from_container(current_state, PLAN.controllers_to_activate, g)
    # print("controllers_to_activate_list: ", controllers_to_activate_list)
    # parse through all controllers to activate

    # transition_list, _ = get_from_container(current_state, PLAN.transitions, g)
    # print("\ttransition_list: ", transition_list)
    # if not transition_list:
    #     print("No transitions found from start state!")
    # for t in transition_list:
    #     triggering_events, _ = get_from_container(t, PLAN.triggering_events, g)
    #     print("\ttriggering_events: ", triggering_events)
    #     for te in triggering_events:
    #         flags, _ = get_from_container(te, MONITOR.flags, g)
    #         print("\t\tflags: ", flags)
    #     next_state = g.value(t, PLAN.state_to_transit_to)
    #     print("\tstate_to_transition_to: ", next_state)

    """
    - start state
    - transitions -> decides which states to add next
    - save corresponding triggering events (flags) and desired values
    - state
    """

    # plant = g.value(subject=None, predicate=rdflib.RDF.type, object=CONTROLLER.Plant)
    # print("plant: ", plant)
    # # print(i)
    # s = g.value(plant, rdflib.RDF.type, None)
    # # print(s)
    # # c = g.objects(i, rdflib.RDF.type)
    # ir_plant = PlantTranslator().translate(g, plant)
    # functions.update(ir_plant["functions"])
    # data_structures.update(ir_plant["data_structures"])

    # for states in plan_set_of_states:
    #     print("states: ", states)
    #     transition_list, _ = get_from_container(states, PLAN.transitions, g)
    #     termination_conditions, _ = get_from_container(states, PLAN.termination, g)
    #     state_parameters, _ = get_from_container(states, PLAN.state_parameters, g)
    #     print("\ttransition_list: ", transition_list)
    #     print("\ttermination_conditions: ", termination_conditions)
    #     print("\tstate_parameters: ", state_parameters)
    #     for tc in termination_conditions:
    #         print("\t\ttermination_condition: ", tc)
    #     if not transition_list:
    #         print("No transitions found from the state: ", states)
    #     for t in transition_list:
    #         triggering_events, _ = get_from_container(t, PLAN.triggering_events, g)
    #         print("\ttriggering_events: ", triggering_events)
    #         for te in triggering_events:
    #             flags, _ = get_from_container(te, MONITOR.flags, g)
    #             desired_flag_value, _ = get_from_container(te, MONITOR.desired_flag_values, g)
    #             print("\t\tflags: ", flags)
    #             print("\t\tdesired_flag_values: ", desired_flag_values)
    #             for flag in flags:
    #                 value_to_monitor = g.value(flag, MONITOR.value_to_monitor)
    #                 reference_value = g.value(flag, MONITOR.reference_value)
    #                 print("\t\t\tvalue_to_monitor: ", value_to_monitor)
    #                 print("\t\t\treference_value: ", reference_value)
    #                 reference_value = g.value(flag, MONITOR.reference_value)
    #                 if (flag, MONITOR.InIntervalMonitor, MONITOR.in_interval_upper_bound) in g:
    #                     in_interval_upper_bound = g.value(flag, MONITOR.in_interval_upper_bound)
    #                     print("\t\t\tin_interval_epsilon: ", in_interval_upper_bound)
    #                 if (flag, MONITOR.InIntervalMonitor, MONITOR.in_interval_lower_bound) in g:
    #                     in_interval_lower_bound = g.value(flag, MONITOR.in_interval_lower_bound)
    #                     print("\t\t\tin_interval_epsilon: ", in_interval_lower_bound)

    #         next_state = g.value(t, PLAN.state_to_transit_to)
    #         print("\tstate_to_transition_to: ", next_state)

    # for o in plan_set_of_states:
    #     # print("object: ", o)
    #     controller_to_load = g.objects(o, PLAN.controllers_to_activate)
    #     for i in controller_to_load:
    #         # print(i)
    #         s = g.value(i, rdflib.RDF.type, None)
    #         # print(s)
    #         # c = g.objects(i, rdflib.RDF.type)
    #         ir1 = PIDControllerTranslator().translate(g, s)
    #         functions.update(ir1["functions"])
    #         data_structures.update(ir1["data_structures"])
    #         # state_machine.update(ir1["state_machine"])
    #         # print("functions: ", functions)
    #         # for j in c:
    #         #     pass
    #         # print(j)

    # ir = {
    #     "functions": functions_dict,
    #     "data_structures": data_structures_dict,
    #     "behaviors": behaviors_dict,
    # }
    # s = g.value(None, rdflib.RDF.type, PID_CONTROLLER.PIDController)

    # # functions[controller] = PIDControllerTranslator().translate(g, s)

    # # json_dict = {
    # #     "constant": g.value(s, p_controller_NS.constant, None),
    # #     "error": g.value(s, p_controller_NS.error, None),
    # #     "gain": g.value(s, p_controller_NS.gain, None),
    # #     "output": g.value(s, p_controller_NS.output, None)
    # # }

    # ir = PIDControllerTranslator().translate(g, s)

    # json_obj = json.dumps(ir, indent=4)

    # print(json_obj)


# if __name__ == "__main__":
#     main()
