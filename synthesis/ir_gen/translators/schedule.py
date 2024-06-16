import rdflib
from namespaces import PLAN, ALGORITHM, MONITOR
from utility import resolver, helper


class ScheduleTranslator:

    def translate(
        self,
        g: rdflib.Graph,
        schedule_call_node_ids,
        behavior_id,
        arm_name: str,
        direction_of_specification: str,
    ) -> dict:
        from ir_gen.translators import (
            FunctionsTranslator,
            MonitorTranslator,
        )

        ir = dict()
        ir["functions"] = dict()
        ir["data_structures"] = dict()
        ir["schedules"] = dict()

        for schedule_call_node_id in schedule_call_node_ids:
            schedule_call_node = g.compute_qname(schedule_call_node_id)[-1]

            types_of_schedule_call_node, _ = helper.get_from_container(
                subject_node=schedule_call_node_id,
                predicate_value=rdflib.RDF.type,
                graph=g,
                return_just_id_after_hash=True,
            )

            if len(types_of_schedule_call_node) == 1:
                if (
                    types_of_schedule_call_node[0]
                    == "ConstraintBasedScheduleSequenceCall"
                ):
                    ir["schedules"][schedule_call_node] = dict()
                    ir["schedules"][schedule_call_node]["trigger_chain"] = []

                    constraint_id = g.value(
                        subject=schedule_call_node_id,
                        predicate=PLAN.constraint_for_schedule,
                    )

                    schedule_id = g.value(
                        subject=schedule_call_node_id, predicate=PLAN.schedule
                    )
                    if schedule_id is None:
                        print(
                            "[Possible Error] [schedule.py] No schedule found for the schedule call node"
                        )
                    schedule_name = g.compute_qname(schedule_id)[-1]

                    trigger_chain_ids, _ = helper.get_from_container(
                        subject_node=schedule_id,
                        predicate_value=ALGORITHM.trigger_chain,
                        graph=g,
                    )
                    trigger_chain_names = [
                        g.compute_qname(x)[-1] for x in trigger_chain_ids
                    ]
                    # trigger_chain_names = [
                    #     x + "_" + schedule_name for x in trigger_chain_names
                    # ]
                    ir["schedules"][schedule_call_node]["trigger_chain"] = trigger_chain_names

                    for function_id in trigger_chain_ids:
                        function_dict = FunctionsTranslator().translate(
                            g=g,
                            function_id=function_id,
                            arm_name=arm_name,
                            direction_of_specification=direction_of_specification,
                            constraint_id=constraint_id,
                        )
                        function_name = (
                            g.compute_qname(function_id)[-1] + "_" + schedule_name
                        )
                        ir["functions"].update(function_dict["functions"])
                        ir["data_structures"].update(function_dict["data_structures"])

                elif (
                    types_of_schedule_call_node[0] == "MonitorBasedScheduleSequenceCall"
                ):
                    print("To be implemented: MonitorBasedScheduleSequenceCall")

                else:
                    print(
                        "[Possible Error] [schedule.py] Unknown type of schedule call node"
                    )

            elif len(types_of_schedule_call_node) == 2:
                if (
                    "ConstraintBasedScheduleSequenceCall" in types_of_schedule_call_node
                    and "MonitorBasedScheduleSequenceCall"
                    in types_of_schedule_call_node
                ):
                    ir["schedules"][schedule_call_node] = dict()
                    ir["schedules"][schedule_call_node]["trigger_chain"] = []
                    ir["schedules"][schedule_call_node]["monitors"] = dict()
                    ir["schedules"][schedule_call_node]["monitors"]["function_names"] = []
                    ir["schedules"][schedule_call_node]["monitors"]["flag_names"] = []

                    constraint_id = g.value(
                        subject=schedule_call_node_id,
                        predicate=PLAN.constraint_for_schedule,
                    )

                    schedule_id = g.value(
                        subject=schedule_call_node_id, predicate=PLAN.schedule
                    )
                    if schedule_id is None:
                        print(
                            "[Possible Error] [schedule.py] No schedule found for the schedule call node"
                        )
                    schedule_name = g.compute_qname(schedule_id)[-1]

                    trigger_chain_ids, _ = helper.get_from_container(
                        subject_node=schedule_id,
                        predicate_value=ALGORITHM.trigger_chain,
                        graph=g,
                    )
                    trigger_chain_names = [
                        g.compute_qname(x)[-1] for x in trigger_chain_ids
                    ]
                    # trigger_chain_names = [
                    #     x + "_" + schedule_name for x in trigger_chain_names
                    # ]
                    ir["schedules"][schedule_call_node][
                        "trigger_chain"
                    ] = trigger_chain_names

                    for function_id in trigger_chain_ids:
                        function_dict = FunctionsTranslator().translate(
                            g=g,
                            function_id=function_id,
                            arm_name=arm_name,
                            direction_of_specification=direction_of_specification,
                            constraint_id=constraint_id,
                        )
                        function_name = (
                            g.compute_qname(function_id)[-1] + "_" + schedule_name
                        )
                        ir["functions"].update(function_dict["functions"])
                        ir["data_structures"].update(function_dict["data_structures"])

                    monitor_for_schedule_id = g.value(
                        subject=schedule_call_node_id,
                        predicate=PLAN.monitors
                    )
                    constraint_to_monitor_id = g.value(
                        subject=monitor_for_schedule_id,
                        predicate=MONITOR.constraint_to_monitor,
                    )
                    
                    ir_monitor = MonitorTranslator().translate(
                        g, constraint_to_monitor_id=constraint_to_monitor_id
                    )
                    ir["functions"].update(ir_monitor["functions"])
                    ir["data_structures"].update(ir_monitor["data_structures"])
                    ir["schedules"][schedule_call_node]["monitors"]["function_names"].append(ir_monitor["function_name"])
                    ir["schedules"][schedule_call_node]["monitors"]["flag_names"].append(ir_monitor["flag_name"])

                else:
                    print(
                        "[Possible Error] [schedule.py] Unknown types of schedule call node found"
                    )
            else:
                print(
                    "[Possible Error] [schedule.py] more than two types of schedule call node found"
                )

        # for given per condition, get all the nodes with "ConstraintBasedScheduleSequenceCall"
        # and if the condition is a predicate "monitor", then get all nodes with "MonitorBasedScheduleSequenceCall"
        # as the type and corersponding constraint associated with it

        # for each of them, get the associated schedule (optionally the monitor)

        return ir
