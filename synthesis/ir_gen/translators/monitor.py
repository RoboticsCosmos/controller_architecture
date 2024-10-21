import rdflib
from namespaces import MONITOR, ALGORITHM
from utility import resolver, helper


class MonitorTranslator:

    def translate(
        self, g: rdflib.Graph, constraint_to_monitor_id, flag_or_event_id, is_flag
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
            "OutOfIntervalConstraint": "out_of_interval_monitor",
            "GreaterThanUpperLimitConstraint": "greater_than_upper_limit_monitor",
            "GreaterThanLowerLimitConstraint": "greater_than_lower_limit_monitor",
            "LowerThanLowerLimitConstraint": "lower_than_lower_limit_monitor",
            "LowerThanUpperLimitConstraint": "lower_than_upper_limit_monitor",
        }

        types_with_only_quantity_to_compare_and_reference_quantity = [
            "EqualConstraint",
            "LessThanConstraint",
            "LessThanEqualToConstraint",
            "GreaterThanConstraint",
            "GreaterThanEqualToConstraint",
        ]

        types_with_tolerance = [
            "InIntervalConstraint",
            "OutOfIntervalConstraint",
            "GreaterThanUpperLimitConstraint",
            "GreaterThanLowerLimitConstraint",
            "LowerThanLowerLimitConstraint",
            "LowerThanUpperLimitConstraint",
        ]

        flag_or_event_name = g.compute_qname(flag_or_event_id)[-1]

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
        constraint_has_tolerance = False
        for type_name in types_of_condition_node:
            if type_name != "Constraint":
                type_of_constraint = type_name
                if type_of_constraint in types_with_tolerance:
                    constraint_has_tolerance = True
                    tolerance_id = g.value(
                        subject=constraint_to_monitor_id, predicate=MONITOR.tolerance
                    )
                    tolerance_name = g.compute_qname(tolerance_id)[-1]

                    ir["data_structures"][tolerance_name] = DataTranslator().translate(
                        g=g, data_id=tolerance_id
                    )
            else:
                pass
        if type_of_constraint is None:
            print("[monitor.py] no type_of_constraint found in the condition node")

        quantity_to_compare_id = g.value(
            subject=constraint_to_monitor_id, predicate=MONITOR.quantity_to_compare
        )
        quantity_to_compare = g.compute_qname(quantity_to_compare_id)[-1]

        reference_quantity_id = g.value(
            subject=constraint_to_monitor_id, predicate=MONITOR.quantity_to_compare_with
        )
        reference_quantity = g.compute_qname(reference_quantity_id)[-1]

        function_name = (
            quantity_to_compare
            + "_"
            + type_of_constraint.lower()
            + "_"
            + reference_quantity
            + "_is_flag_"
            + str(is_flag)
            + "_function"
        )

        # flag_name = (
        #     quantity_to_compare
        #     + "_"
        #     + type_of_constraint.lower()
        #     + "_"
        #     + reference_quantity
        #     + "_flag"
        # )

        ir["functions"][function_name] = dict()
        ir["functions"][function_name]["type_of_function"] = type_of_constraint
        ir["functions"][function_name]["function_call"] = type_name_function_map[
            type_of_constraint
        ]
        ir["functions"][function_name]["quantity_to_compare"] = quantity_to_compare
        ir["functions"][function_name]["reference_quantity"] = reference_quantity
        ir["functions"][function_name]["flag_name"] = flag_or_event_name

        ir["data_structures"][quantity_to_compare] = DataTranslator().translate(
            g=g, data_id=quantity_to_compare_id
        )
        ir["data_structures"][reference_quantity] = DataTranslator().translate(
            g=g, data_id=reference_quantity_id
        )

        ir["data_structures"][flag_or_event_name] = DataTranslator().translate(
            g=g, data_id=flag_or_event_id
        )
        ir["function_name"] = function_name
        ir["flag_name"] = flag_or_event_name

        if constraint_has_tolerance:
            ir["functions"][function_name]["tolerance"] = tolerance_name
            ir["data_structures"][tolerance_name] = DataTranslator().translate(
                g=g, data_id=tolerance_id
            )
        return ir
