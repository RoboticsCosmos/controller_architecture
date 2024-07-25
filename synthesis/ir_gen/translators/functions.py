import rdflib
from namespaces import FUNCTIONS, PLAN, ALGORITHM, ERROR, MONITOR
from utility import resolver, helper


class FunctionsTranslator:

    def translate(
        self,
        g: rdflib.Graph,
        function_id,
    ) -> dict:
        from ir_gen.translators import (
            DataTranslator,
        )

        ir = dict()
        ir["functions"] = dict()
        ir["data_structures"] = dict()

        function_name = g.compute_qname(function_id)[-1]

        types_of_function_node, _ = helper.get_from_container(
            subject_node=function_id,
            predicate_value=rdflib.RDF.type,
            graph=g,
            return_just_id_after_hash=True,
        )

        # SUM: arguments_list, summed_data
        # summation2, summation3
        if "Sum" in types_of_function_node:
            input_arguments_id, _ = helper.get_from_container(
                subject_node=function_id,
                predicate_value=FUNCTIONS.arguments_list,
                graph=g,
            )
            input_arguments_names = [
                g.compute_qname(arg)[-1] for arg in input_arguments_id
            ]

            for argument_id in input_arguments_id:
                argument_name = g.compute_qname(argument_id)[-1]
                ir["data_structures"][argument_name] = DataTranslator().translate(
                    g=g, data_id=argument_id
                )

            summed_data_id = g.value(
                subject=function_id, predicate=FUNCTIONS.summed_data
            )
            argument_name = g.compute_qname(summed_data_id)[-1]
            ir["data_structures"][argument_name] = DataTranslator().translate(
                g=g, data_id=summed_data_id
            )

            function_call_name = ""
            if len(input_arguments_names) == 2:
                function_call_name = "summation2"
            elif len(input_arguments_names) == 3:
                function_call_name = "summation3"
            else:
                print(
                    "[functions.py] [Possible Error] Number of input arguments not supported for summation"
                )

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["arguments"] = input_arguments_names
            ir["functions"][function_name]["summed_data"] = g.compute_qname(
                summed_data_id
            )[-1]

        # SUBTRACT: arguments_list, difference_data
        # subtraction(const double *start_value, const double *reduction_value, double *difference)
        if "Subtract" in types_of_function_node:
            function_call_name = "subtraction"
            input_arguments_id, _ = helper.get_from_container(
                subject_node=function_id,
                predicate_value=FUNCTIONS.arguments_list,
                graph=g,
            )
            input_arguments_names = [
                g.compute_qname(arg)[-1] for arg in input_arguments_id
            ]

            for argument_id in input_arguments_id:
                argument_name = g.compute_qname(argument_id)[-1]
                ir["data_structures"][argument_name] = DataTranslator().translate(
                    g=g, data_id=argument_id
                )

            difference_data_id = g.value(
                subject=function_id, predicate=FUNCTIONS.difference_data
            )
            argument_name = g.compute_qname(difference_data_id)[-1]
            ir["data_structures"][argument_name] = DataTranslator().translate(
                g=g, data_id=difference_data_id
            )

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["arguments"] = input_arguments_names
            ir["functions"][function_name]["difference_data"] = g.compute_qname(
                difference_data_id
            )[-1]

        # ErrorBasedOnConstraint, InstantaneousError: error_data
        # subtraction(const double *start_value, const double *reduction_value, double *difference)
        if "ErrorFunction" in types_of_function_node:
            if "InstantaneousError" in types_of_function_node:
                function_call_name = "subtraction"

                # getting attributes of the constraint
                quantity_to_compare_id = g.value(
                    subject=function_id, predicate=ERROR.measured_quantity
                )
                quantity_to_compare_name = g.compute_qname(quantity_to_compare_id)[-1]
                reference_quantity_id = g.value(
                    subject=function_id, predicate=ERROR.reference_quantity
                )
                reference_quantity_name = g.compute_qname(reference_quantity_id)[-1]

                # (first-second) -> (desired-measured)
                input_arguments_id = [reference_quantity_id, quantity_to_compare_id]
                input_arguments_names = [
                    g.compute_qname(arg)[-1] for arg in input_arguments_id
                ]

                for argument_id in input_arguments_id:
                    argument_name = g.compute_qname(argument_id)[-1]
                    ir["data_structures"][argument_name] = DataTranslator().translate(
                        g=g, data_id=argument_id
                    )

                difference_data_id = g.value(
                    subject=function_id, predicate=ERROR.error_data
                )
                argument_name = g.compute_qname(difference_data_id)[-1]
                ir["data_structures"][argument_name] = DataTranslator().translate(
                    g=g, data_id=difference_data_id
                )

                error_data_id = g.value(subject=function_id, predicate=ERROR.error_data)
                error_data_name = g.compute_qname(error_data_id)[-1]

                ir["data_structures"][error_data_name] = DataTranslator().translate(
                    g=g, data_id=error_data_id
                )

                ir["functions"][function_name] = dict()
                ir["functions"][function_name]["function_call"] = function_call_name
                ir["functions"][function_name]["arguments"] = input_arguments_names
                ir["functions"][function_name]["difference_data"] = error_data_name
            else:
                print(
                    "[functions.py] [Possible Error] ErrorBasedOnConstraint function without InstantaneousError"
                )

        # Scale: arguments_list, product_data
        # multiply2, multiply3
        if "Scale" in types_of_function_node:
            input_arguments_id, _ = helper.get_from_container(
                subject_node=function_id,
                predicate_value=FUNCTIONS.arguments_list,
                graph=g,
            )
            input_arguments_names = [
                g.compute_qname(arg)[-1] for arg in input_arguments_id
            ]

            for argument_id in input_arguments_id:
                argument_name = g.compute_qname(argument_id)[-1]
                ir["data_structures"][argument_name] = DataTranslator().translate(
                    g=g, data_id=argument_id
                )

            product_data_id = g.value(
                subject=function_id, predicate=FUNCTIONS.product_data
            )
            argument_name = g.compute_qname(product_data_id)[-1]
            ir["data_structures"][argument_name] = DataTranslator().translate(
                g=g, data_id=product_data_id
            )

            function_call_name = ""
            if len(input_arguments_names) == 2:
                function_call_name = "multiply2"
            elif len(input_arguments_names) == 3:
                function_call_name = "multiply3"
            else:
                print(
                    "[functions.py] [Possible Error] Number of input arguments not supported for multiplication"
                )

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["arguments"] = input_arguments_names
            ir["functions"][function_name]["product_data"] = g.compute_qname(
                product_data_id
            )[-1]

        # DerivativeFromError: time_period, current_value, previous_value, differentiated_data
        # void differentiator(const double *current_value, const double *previous_value, const double *dt, double *output)
        if "DerivativeFromError" in types_of_function_node:
            function_call_name = "differentiator"

            # getting all attributes
            time_period_id = g.value(subject=function_id, predicate=ERROR.time_period)
            time_period_name = g.compute_qname(time_period_id)[-1]

            current_value_id = g.value(
                subject=function_id, predicate=ERROR.current_value
            )

            previous_value_id = g.value(
                subject=function_id, predicate=ERROR.previous_value
            )

            current_value_name = g.compute_qname(current_value_id)[-1]
            previous_value_name = g.compute_qname(previous_value_id)[-1]

            differentiated_data_id = g.value(
                subject=function_id, predicate=ERROR.differentiated_data
            )
            differentiated_data_name = g.compute_qname(differentiated_data_id)[-1]

            # updating the data structures
            ir["data_structures"][time_period_name] = DataTranslator().translate(
                g=g, data_id=time_period_id
            )
            ir["data_structures"][current_value_name] = DataTranslator().translate(
                g=g, data_id=current_value_id
            )
            ir["data_structures"][previous_value_name] = DataTranslator().translate(
                g=g, data_id=previous_value_id
            )
            ir["data_structures"][
                differentiated_data_name
            ] = DataTranslator().translate(g=g, data_id=differentiated_data_id)

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["time_period"] = time_period_name
            ir["functions"][function_name]["current_value"] = current_value_name
            ir["functions"][function_name]["previous_value"] = previous_value_name
            ir["functions"][function_name][
                "differentiated_data"
            ] = differentiated_data_name

        # IntegralFromStartError: time_period, quantity_to_integrate, integrated_data
        # integrator(const double *input, const double *dt, double *output)
        if "IntegralFromStartError" in types_of_function_node:
            function_call_name = "integrator"

            # getting all attributes
            time_period_id = g.value(subject=function_id, predicate=ERROR.time_period)
            time_period_name = g.compute_qname(time_period_id)[-1]

            quantity_to_integrate_id = g.value(
                subject=function_id, predicate=ERROR.quantity_to_integrate
            )
            quantity_to_integrate_name = g.compute_qname(quantity_to_integrate_id)[-1]

            integrated_data_id = g.value(
                subject=function_id, predicate=ERROR.integrated_data
            )
            integrated_data_name = g.compute_qname(integrated_data_id)[-1]

            # updating the data structures
            ir["data_structures"][time_period_name] = DataTranslator().translate(
                g=g, data_id=time_period_id
            )
            ir["data_structures"][
                quantity_to_integrate_name
            ] = DataTranslator().translate(g=g, data_id=quantity_to_integrate_id)
            ir["data_structures"][integrated_data_name] = DataTranslator().translate(
                g=g, data_id=integrated_data_id
            )

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["time_period"] = time_period_name
            ir["functions"][function_name][
                "quantity_to_integrate"
            ] = quantity_to_integrate_name
            ir["functions"][function_name]["integrated_data"] = integrated_data_name

        # Saturation: signal_to_saturate, saturated_data, saturation_limits
        # saturation(double *input, const double *lower_limit, const double *upper_limit)
        if "Saturation" in types_of_function_node:
            function_call_name = "saturation"

            # getting all attributes
            signal_to_saturate_id = g.value(
                subject=function_id, predicate=FUNCTIONS.signal_to_saturate
            )
            signal_to_saturate_name = g.compute_qname(signal_to_saturate_id)[-1]

            saturated_data_id = g.value(
                subject=function_id, predicate=FUNCTIONS.saturated_data
            )
            saturated_data_name = g.compute_qname(saturated_data_id)[-1]

            saturation_limits_ids, _ = helper.get_from_container(
                subject_node=function_id,
                predicate_value=FUNCTIONS.saturation_limits,
                graph=g,
            )

            [saturation_lower_limit_id, saturation_upper_limit_id] = (
                saturation_limits_ids
            )
            saturation_limit_names = [
                g.compute_qname(x)[-1] for x in saturation_limits_ids
            ]
            saturation_lower_limit_name = saturation_limit_names[0]
            saturation_upper_limit_name = saturation_limit_names[1]

            # updating the data structures
            ir["data_structures"][signal_to_saturate_name] = DataTranslator().translate(
                g=g, data_id=signal_to_saturate_id
            )
            ir["data_structures"][saturated_data_name] = DataTranslator().translate(
                g=g, data_id=saturated_data_id
            )
            ir["data_structures"][
                saturation_lower_limit_name
            ] = DataTranslator().translate(g=g, data_id=saturation_lower_limit_id)
            ir["data_structures"][
                saturation_upper_limit_name
            ] = DataTranslator().translate(g=g, data_id=saturation_upper_limit_id)

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name][
                "signal_to_saturate"
            ] = signal_to_saturate_name
            ir["functions"][function_name]["saturated_data"] = saturated_data_name
            ir["functions"][function_name]["saturation_limits"] = saturation_limit_names

        # GetSign: input_signal, sign_data
        # get_sign(const double *value, double *sign)
        if "GetSign" in types_of_function_node:
            function_call_name = "get_sign"

            # getting all attributes
            input_signal_id = g.value(subject=function_id, predicate=FUNCTIONS.input_signal)
            input_signal_name = g.compute_qname(input_signal_id)[-1]

            sign_data_id = g.value(subject=function_id, predicate=FUNCTIONS.sign_data)
            sign_data_name = g.compute_qname(sign_data_id)[-1]

            # updating the data structures
            ir["data_structures"][input_signal_name] = DataTranslator().translate(
                g=g, data_id=input_signal_id
            )
            ir["data_structures"][sign_data_name] = DataTranslator().translate(
                g=g, data_id=sign_data_id
            )

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["input_signal"] = input_signal_name
            ir["functions"][function_name]["sign_data"] = sign_data_name

        # HeavisideStepFunction: input_signal, stepped_data
        # hside(const double *value, double *hside_output)
        if "HeavisideStepFunction" in types_of_function_node:
            function_call_name = "hside"

            # getting all attributes
            input_signal_id = g.value(subject=function_id, predicate=FUNCTIONS.input_signal)
            input_signal_name = g.compute_qname(input_signal_id)[-1]

            stepped_data_id = g.value(subject=function_id, predicate=FUNCTIONS.stepped_data)
            stepped_data_name = g.compute_qname(stepped_data_id)[-1]

            # updating the data structures
            ir["data_structures"][input_signal_name] = DataTranslator().translate(
                g=g, data_id=input_signal_id
            )
            ir["data_structures"][stepped_data_name] = DataTranslator().translate(
                g=g, data_id=stepped_data_id
            )

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["input_signal"] = input_signal_name
            ir["functions"][function_name]["stepped_data"] = stepped_data_name

        # GetAbs: input_signal, absolute_data
        # get_abs(const double *value, double *abs_output)
        if "GetAbs" in types_of_function_node:
            function_call_name = "get_abs"

            # getting all attributes
            input_signal_id = g.value(
                subject=function_id, predicate=FUNCTIONS.input_signal
            )
            input_signal_name = g.compute_qname(input_signal_id)[-1]

            absolute_data_id = g.value(
                subject=function_id, predicate=FUNCTIONS.absolute_data
            )
            absolute_data_name = g.compute_qname(absolute_data_id)[-1]

            # updating the data structures
            ir["data_structures"][input_signal_name] = DataTranslator().translate(
                g=g, data_id=input_signal_id
            )
            ir["data_structures"][absolute_data_name] = DataTranslator().translate(
                g=g, data_id=absolute_data_id
            )

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name]["input_signal"] = input_signal_name
            ir["functions"][function_name]["absolute_data"] = absolute_data_name

        # CopyVariableValue: variable_to_copy_from, variable_to_copy_to
        # set_value_of_first_to_second_variable(const double *first, double *second)
        if "CopyVariableValue" in types_of_function_node:
            function_call_name = "set_value_of_first_to_second_variable"

            # getting all attributes
            first_variable_to_copy_from_id = g.value(
                subject=function_id, predicate=FUNCTIONS.variable_to_copy_from
            )
            first_variable_name = g.compute_qname(first_variable_to_copy_from_id)[-1]

            second_variable_to_copy_to_id = g.value(
                subject=function_id, predicate=FUNCTIONS.variable_to_copy_to
            )
            second_variable_name = g.compute_qname(second_variable_to_copy_to_id)[-1]

            # updating the data structures
            ir["data_structures"][first_variable_name] = DataTranslator().translate(
                g=g, data_id=first_variable_to_copy_from_id
            )
            ir["data_structures"][second_variable_name] = DataTranslator().translate(
                g=g, data_id=second_variable_to_copy_to_id
            )

            # updating the functions

            ir["functions"][function_name] = dict()
            ir["functions"][function_name]["function_call"] = function_call_name
            ir["functions"][function_name][
                "first_variable_to_copy_from"
            ] = first_variable_name
            ir["functions"][function_name][
                "second_variable_to_copy_to"
            ] = second_variable_name

        return ir
