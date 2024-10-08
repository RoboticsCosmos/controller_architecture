import rdflib
from namespaces import PLAN, ALGORITHM, MONITOR
from utility import resolver, helper

# TODO: get Activity as input to this translator, then get all the schedule_call nodes.


# assumption: for every per-condition, there can be more than one controller, but in separate constraint ctr nodes; for every schedule, theere can be more than one sch_evnt_cbk
# assumption: for every per-condition, one activity is associated
class ScheduleTranslator:

    def translate(
        self,
        g: rdflib.Graph,
        activity_ids,
        motion_spec_id,
    ) -> dict:
        from ir_gen.translators import (
            FunctionsTranslator,
            MonitorTranslator,
            DataTranslator,
        )

        ir = dict()
        ir["functions"] = dict()
        ir["data_structures"] = dict()
        ir["schedules"] = dict()

        for activity_id in activity_ids:
            schedule_call_ids, _ = helper.get_from_container(
                subject_node=activity_id,
                predicate_value=ALGORITHM.sch_evnt_assn,
                graph=g,
            )

            if len(schedule_call_ids) == 0:
                print(
                    "[Possible Error] [schedule.py] No schedule call found for the activity"
                )

            for schedule_call_node_id in schedule_call_ids:

                schedule_call_node_name = g.compute_qname(schedule_call_node_id)[-1]

                schedule_node_id = g.value(
                    subject=schedule_call_node_id,
                    predicate=ALGORITHM.schedule_to_call,
                )

                types_of_schedule_call_node, _ = helper.get_from_container(
                    subject_node=schedule_call_node_id,
                    predicate_value=rdflib.RDF.type,
                    graph=g,
                    return_just_id_after_hash=True,
                )

                if "EventBasedScheduleCallback" in types_of_schedule_call_node:
                    ir["schedules"][schedule_call_node_name] = dict()
                    ir["schedules"][schedule_call_node_name]["trigger_chain"] = []
                    ir["schedules"][schedule_call_node_name]["monitors"] = dict()
                    ir["schedules"][schedule_call_node_name]["monitors"][
                        "function_names"
                    ] = []
                    ir["schedules"][schedule_call_node_name]["monitors"][
                        "event_data"
                    ] = []

                    desired_events_id_list, _ = helper.get_from_container(
                        subject_node=schedule_call_node_id,
                        predicate_value=ALGORITHM.desired_events,
                        graph=g,
                    )

                    trigger_chain_ids, _ = helper.get_from_container(
                        subject_node=schedule_node_id,
                        predicate_value=ALGORITHM.trigger_chain,
                        graph=g,
                    )
                    trigger_chain_names = [
                        g.compute_qname(x)[-1] for x in trigger_chain_ids
                    ]

                    ir["schedules"][schedule_call_node_name][
                        "trigger_chain"
                    ] = trigger_chain_names

                    for function_id in trigger_chain_ids:
                        function_dict = FunctionsTranslator().translate(
                            g=g,
                            function_id=function_id,
                        )
                        ir["functions"].update(function_dict["functions"])
                        ir["data_structures"].update(function_dict["data_structures"])

                    for desired_event_id in desired_events_id_list:

                        event_name = g.compute_qname(desired_event_id)[-1]
                        ir["data_structures"][event_name] = DataTranslator().translate(
                            g=g,
                            data_id=desired_event_id,
                        )

                        constraint_monitor_id = g.value(
                            predicate=MONITOR.event_emitted,
                            object=desired_event_id,
                        )

                        constraint_to_monitor_id = g.value(
                            subject=constraint_monitor_id,
                            predicate=MONITOR.constraint_to_monitor,
                        )

                        ir_monitor = MonitorTranslator().translate(
                            g,
                            constraint_to_monitor_id=constraint_to_monitor_id,
                            flag_or_event_id=desired_event_id,
                            is_flag=False,
                        )
                        ir["functions"].update(ir_monitor["functions"])
                        ir["data_structures"].update(ir_monitor["data_structures"])
                        ir["schedules"][schedule_call_node_name]["monitors"][
                            "function_names"
                        ].append(ir_monitor["function_name"])
                        ir["schedules"][schedule_call_node_name]["monitors"][
                            "event_data"
                        ].append(ir_monitor["flag_name"])

                else:
                    print(
                        "[Possible Error] [schedule.py] Unknown type of schedule call node"
                    )

                    motion_spec_name = g.compute_qname(motion_spec_id)[-1]
                    print("types_of_schedule_call_node: ", types_of_schedule_call_node)
                    print("motion_spec_name: ", motion_spec_name)
                    print("activity name: ", g.compute_qname(activity_id)[-1])

        return ir

        # class ScheduleTranslator:

        #     def translate(
        #         self,
        #         g: rdflib.Graph,
        #         schedule_node_ids,
        #         motion_spec_id,
        #     ) -> dict:
        #         from ir_gen.translators import (
        #             FunctionsTranslator,
        #             MonitorTranslator,
        #             DataTranslator,
        #         )

        #         ir = dict()
        #         ir["functions"] = dict()
        #         ir["data_structures"] = dict()
        #         ir["schedules"] = dict()

        #         for schedule_node_id in schedule_node_ids:

        #             # get coresponding schedule_call
        #             schedule_call_node_id = g.value(
        #                 predicate=ALGORITHM.schedule_to_call, object=schedule_node_id
        #             )
        #             if schedule_call_node_id is None:
        #                 print(
        #                     "[Possible Error] [schedule.py] No schedule call found for the schedule node"
        #                 )
        #                 continue

        #             schedule_call_node_name = g.compute_qname(schedule_call_node_id)[-1]
        #             schedule_node_name = g.compute_qname(schedule_node_id)[-1]

        #             types_of_schedule_call_node, _ = helper.get_from_container(
        #                 subject_node=schedule_call_node_id,
        #                 predicate_value=rdflib.RDF.type,
        #                 graph=g,
        #                 return_just_id_after_hash=True,
        #             )

        #             if "EventBasedScheduleCallback" in types_of_schedule_call_node:
        #                 ir["schedules"][schedule_call_node_name] = dict()
        #                 ir["schedules"][schedule_call_node_name]["trigger_chain"] = []
        #                 ir["schedules"][schedule_call_node_name]["monitors"] = dict()
        #                 ir["schedules"][schedule_call_node_name]["monitors"][
        #                     "function_names"
        #                 ] = []
        #                 ir["schedules"][schedule_call_node_name]["monitors"]["event_data"] = []

        #                 desired_events_id_list, _ = helper.get_from_container(
        #                     subject_node=schedule_call_node_id,
        #                     predicate_value=ALGORITHM.desired_events,
        #                     graph=g,
        #                 )

        #                 trigger_chain_ids, _ = helper.get_from_container(
        #                     subject_node=schedule_node_id,
        #                     predicate_value=ALGORITHM.trigger_chain,
        #                     graph=g,
        #                 )
        #                 trigger_chain_names = [
        #                     g.compute_qname(x)[-1] for x in trigger_chain_ids
        #                 ]

        #                 ir["schedules"][schedule_call_node_name][
        #                     "trigger_chain"
        #                 ] = trigger_chain_names

        #                 for function_id in trigger_chain_ids:
        #                     function_dict = FunctionsTranslator().translate(
        #                         g=g,
        #                         function_id=function_id,
        #                     )
        #                     ir["functions"].update(function_dict["functions"])
        #                     ir["data_structures"].update(function_dict["data_structures"])

        #                 for desired_event_id in desired_events_id_list:

        #                     event_name = g.compute_qname(desired_event_id)[-1]
        #                     ir["data_structures"][event_name] = DataTranslator().translate(
        #                         g=g,
        #                         data_id=desired_event_id,
        #                     )

        #                     constraint_monitor_id = g.value(
        #                         predicate=MONITOR.event_emitted,
        #                         object=desired_event_id,
        #                     )

        #                     constraint_to_monitor_id = g.value(
        #                         subject=constraint_monitor_id,
        #                         predicate=MONITOR.constraint_to_monitor,
        #                     )

        #                     ir_monitor = MonitorTranslator().translate(
        #                         g,
        #                         constraint_to_monitor_id=constraint_to_monitor_id,
        #                         flag_or_event_id=desired_event_id,
        #                         is_flag=False,
        #                     )
        #                     ir["functions"].update(ir_monitor["functions"])
        #                     ir["data_structures"].update(ir_monitor["data_structures"])
        #                     ir["schedules"][schedule_call_node_name]["monitors"][
        #                         "function_names"
        #                     ].append(ir_monitor["function_name"])
        #                     ir["schedules"][schedule_call_node_name]["monitors"][
        #                         "event_data"
        #                     ].append(ir_monitor["flag_name"])

        #             else:
        #                 print(
        #                     "[Possible Error] [schedule.py] Unknown type of schedule call node"
        #                 )

        # end #

        # elif len(types_of_schedule_call_node) == 2:
        #     if (
        #         "FlagsBasedScheduleCallback" in types_of_schedule_call_node
        #         and "MonitorBasedScheduleSequenceCall"
        #         in types_of_schedule_call_node
        #     ):
        #         ir["schedules"][schedule_call_node] = dict()
        #         ir["schedules"][schedule_call_node]["trigger_chain"] = []
        #         ir["schedules"][schedule_call_node]["monitors"] = dict()
        #         ir["schedules"][schedule_call_node]["monitors"][
        #             "function_names"
        #         ] = []
        #         ir["schedules"][schedule_call_node]["monitors"]["flag_names"] = []

        #         constraint_id = g.value(
        #             subject=schedule_call_node_id,
        #             predicate=PLAN.flags_to_check,
        #         )

        #         schedule_id = g.value(
        #             subject=schedule_call_node_id, predicate=PLAN.schedule
        #         )
        #         if schedule_id is None:
        #             print(
        #                 "[Possible Error] [schedule.py] No schedule found for the schedule call node"
        #             )
        #         schedule_name = g.compute_qname(schedule_id)[-1]

        #         trigger_chain_ids, _ = helper.get_from_container(
        #             subject_node=schedule_id,
        #             predicate_value=ALGORITHM.trigger_chain,
        #             graph=g,
        #         )
        #         trigger_chain_names = [
        #             g.compute_qname(x)[-1] for x in trigger_chain_ids
        #         ]
        #         # trigger_chain_names = [
        #         #     x + "_" + schedule_name for x in trigger_chain_names
        #         # ]
        #         ir["schedules"][schedule_call_node][
        #             "trigger_chain"
        #         ] = trigger_chain_names

        #         for function_id in trigger_chain_ids:
        #             function_dict = FunctionsTranslator().translate(
        #                 g=g,
        #                 function_id=function_id,
        #                 arm_name=arm_name,
        #                 direction_of_specification=direction_of_specification,
        #                 constraint_id=constraint_id,
        #             )
        #             function_name = (
        #                 g.compute_qname(function_id)[-1] + "_" + schedule_name
        #             )
        #             ir["functions"].update(function_dict["functions"])
        #             ir["data_structures"].update(function_dict["data_structures"])

        #         monitor_for_schedule_id = g.value(
        #             subject=schedule_call_node_id, predicate=PLAN.monitors
        #         )
        #         constraint_to_monitor_id = g.value(
        #             subject=monitor_for_schedule_id,
        #             predicate=MONITOR.constraint_to_monitor,
        #         )

        #         ir_monitor = MonitorTranslator().translate(
        #             g, constraint_to_monitor_id=constraint_to_monitor_id
        #         )
        #         ir["functions"].update(ir_monitor["functions"])
        #         ir["data_structures"].update(ir_monitor["data_structures"])
        #         ir["schedules"][schedule_call_node]["monitors"][
        #             "function_names"
        #         ].append(ir_monitor["function_name"])
        #         ir["schedules"][schedule_call_node]["monitors"][
        #             "flag_names"
        #         ].append(ir_monitor["flag_name"])

        #     else:
        #         print(
        #             "[Possible Error] [schedule.py] Unknown types of schedule call node found"
        #         )
        # else:
        #     print(
        #         "[Possible Error] [schedule.py] more than two types of schedule call node found"
        #     )

        # for given per condition, get all the nodes with "FlagsBasedScheduleCallback"
        # and if the condition is a predicate "monitor", then get all nodes with "MonitorBasedScheduleSequenceCall"
        # as the type and corersponding constraint associated with it

        # for each of them, get the associated schedule (optionally the monitor)
