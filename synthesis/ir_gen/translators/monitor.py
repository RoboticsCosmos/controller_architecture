import rdflib
from namespaces import MONITOR, ALGORITHM
from utility import resolver, helper


class MonitorTranslator:

    def translate(
        self,
        g: rdflib.Graph,
        constraint_to_monitor_id,
    ) -> dict:
        from ir_gen.translators import (
            DataTranslator,
        )

        type_name_function_map = {
            "EqualConstraint": "equality_monitor",
            "LessThanConstraint": "less_than_monitor",
            "LessThanEqualToConstraint": "less_than_equal_to_monitor",
            "GreaterThanConstraint": "greater_than_monitor",
            "GreaterThanEqualToConstraint": "greater_than_equal_to_monitor",
            "InIntervalConstraint": "in_interval_monitor",
        }

        ir = dict()
        ir["functions"] = dict()
        ir["data_structures"] = dict()
        ir["function_name"] = []
        ir["flag_name"] = []

        types_of_condition_node, _ = helper.get_from_container(
            subject_node=constraint_to_monitor_id,
            predicate_value=rdflib.RDF.type,
            graph=g,
            return_just_id_after_hash=True,
        )
        type_of_constraint = None
        for type_name in types_of_condition_node:
            if type_name != "Constraint":
                type_of_constraint
            else:
                type_of_constraint = type_name
        if type_of_constraint is None:
            print("[monitor.py] no type_of_constraint found in the condition node")

        quantity_to_compare_id = g.value(
            subject=constraint_to_monitor_id, predicate=MONITOR.quantity_to_compare
        )
        quantity_to_compare = g.compute_qname(quantity_to_compare_id)[-1]

        comparison_types_one_sided = [
            "EqualConstraint",
            "LessThanConstraint",
            "LessThanEqualToConstraint",
            "GreaterThanConstraint",
            "GreaterThanEqualToConstraint",
        ]
        comparison_types_two_sided = ["InIntervalConstraint"]
        if type_name in comparison_types_one_sided:
            reference_quantity_id = g.value(
                subject=constraint_to_monitor_id, predicate=MONITOR.reference_quantity
            )
            reference_quantity = g.compute_qname(reference_quantity_id)[-1]

            function_name = (
                quantity_to_compare
                + "_"
                + type_name.lower()
                + "_"
                + reference_quantity
                + "_function"
            )

            flag_name = (
                quantity_to_compare
                + "_"
                + type_name.lower()
                + "_"
                + reference_quantity
                + "_flag"
            )

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["type_of_function"] = type_name
            ir["functions"][function_name]["function_call"] = type_name_function_map[
                type_name
            ]
            ir["functions"][function_name]["quantity_to_compare"] = quantity_to_compare
            ir["functions"][function_name]["reference_quantity"] = reference_quantity
            ir["functions"][function_name]["flag_name"] = flag_name

            ir["data_structures"][quantity_to_compare] = DataTranslator().translate(
                g=g, data_id=quantity_to_compare_id
            )
            ir["data_structures"][reference_quantity] = DataTranslator().translate(
                g=g, data_id=reference_quantity_id
            )

            ir["data_structures"][flag_name] = dict()
            ir["data_structures"][flag_name]["data_type"] = "bool"
            ir["data_structures"][flag_name]["initial_value"] = "false"

        if type_name in comparison_types_two_sided:
            in_interval_upper_bound_id = g.value(
                subject=constraint_to_monitor_id, predicate=MONITOR.in_interval_upper_bound
            )
            in_interval_lower_bound_id = g.value(
                subject=constraint_to_monitor_id, predicate=MONITOR.in_interval_lower_bound
            )

            in_interval_upper_bound = g.compute_qname(in_interval_upper_bound_id)[-1]
            in_interval_lower_bound = g.compute_qname(in_interval_lower_bound_id)[-1]
            function_name = (
                quantity_to_compare
                + "_"
                + type_name.lower()
                + "_"
                + in_interval_lower_bound
                + "_and_"
                + in_interval_upper_bound
                + "_function"
            )

            flag_name = (
                quantity_to_compare
                + "_"
                + type_name.lower()
                + "_"
                + in_interval_lower_bound
                + "_and_"
                + in_interval_upper_bound
                + "_flag"
            )

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["type_of_function"] = type_name
            ir["functions"][function_name]["function_call"] = type_name_function_map[
                type_name
            ]
            ir["functions"][function_name]["quantity_to_compare"] = quantity_to_compare
            ir["functions"][function_name][
                "in_interval_lower_bound"
            ] = in_interval_lower_bound
            ir["functions"][function_name][
                "in_interval_upper_bound"
            ] = in_interval_upper_bound
            ir["functions"][function_name]["flag_name"] = flag_name

            ir["data_structures"][quantity_to_compare] = DataTranslator().translate(
                g=g, data_id=quantity_to_compare_id
            )
            ir["data_structures"][in_interval_lower_bound] = DataTranslator().translate(
                g=g, data_id=in_interval_lower_bound_id
            )
            ir["data_structures"][in_interval_upper_bound] = DataTranslator().translate(
                g=g, data_id=in_interval_upper_bound_id
            )

            ir["data_structures"][flag_name] = dict()
            ir["data_structures"][flag_name]["data_type"] = "bool"
            ir["data_structures"][flag_name]["initial_value"] = "false"

        ir["function_name"] = function_name
        ir["flag_name"] = flag_name

        return ir