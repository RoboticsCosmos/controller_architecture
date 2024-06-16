import rdflib
from namespaces import CONTROLLER
from utility import resolver


class PlantTranslator:
    """
    currently not used
    """

    def translate(self, g: rdflib.Graph, node) -> dict:
        functions = dict()
        data_structures = dict()
        state_machine = dict()

        a = g.value(subject=None, predicate=rdflib.RDF.type, object=CONTROLLER.Plant)
        plant = g.compute_qname(a)[-1]

        g.predicates(subject=a, object=None)
        input_list = []

        for i in g.predicates(subject=a, object=None):
            key_name = g.compute_qname(i)[-1]
            if key_name != "type" and key_name != "container":
                input_list.append(key_name)

        functions[plant] = dict()
        functions[plant]["name"] = plant
        functions[plant]["control_mode"] = "control_mode"
        functions[plant]["servoing_mode"] = "servoing_mode"
        actuation_signal_id = g.value(subject=a, predicate=CONTROLLER.actuation_signal)
        functions[plant]["actuation_signal"] = g.value(
            subject=actuation_signal_id, predicate=CONTROLLER.init_value
        )
        functions[plant]["measured_pos"] = "measured_pos_value"
        functions[plant]["measured_vel"] = "measured_vel_value"
        functions[plant]["return"] = None

        data_structures["measured_pos_value"] = dict()
        data_structures["measured_pos_value"]["d_type"] = "double"
        measured_pos_signal_node = g.value(subject=a, predicate=CONTROLLER.measured_pos)
        data_structures["measured_pos_value"]["init_val"] = g.value(
            subject=measured_pos_signal_node,
            predicate=CONTROLLER.init_value,
        )

        data_structures["measured_vel_value"] = dict()
        data_structures["measured_vel_value"]["d_type"] = "double"
        measured_vel_signal_node = g.value(subject=a, predicate=CONTROLLER.measured_vel)
        data_structures["measured_vel_value"]["init_val"] = g.value(
            subject=measured_vel_signal_node,
            predicate=CONTROLLER.init_value,
        )

        actuation_signal_node = g.value(
            subject=a, predicate=CONTROLLER.actuation_signal
        )
        data_structures["controller_output_value"] = dict()
        data_structures["controller_output_value"]["d_type"] = "double"
        data_structures["controller_output_value"]["init_val"] = g.value(
            subject=actuation_signal_node,
            predicate=CONTROLLER.init_value,
        )

        data_structures["control_mode"] = dict()
        data_structures["control_mode"]["d_type"] = "string"
        data_structures["control_mode"]["value"] = g.value(
            subject=a, predicate=CONTROLLER.control_mode
        )

        data_structures["servoing_mode"] = dict()
        data_structures["servoing_mode"]["d_type"] = "string"
        data_structures["servoing_mode"]["value"] = g.value(
            subject=a, predicate=CONTROLLER.servoing_mode
        )

        return {
            "functions": functions,
            "state_machine": state_machine,
            "data_structures": data_structures,
        }


class ControllerTranslator:
    """
    currently not used
    """
    def translate(self, g: rdflib.Graph, node) -> dict:

        functions = dict()
        data_structures = dict()
        for a in g.subjects(rdflib.RDF.type, CONTROLLER.Controller):
            controller = a.split("#")[-1]
            print(controller)
            functions[controller] = dict()
            functions[controller]["name"] = controller
            functions[controller]["error"] = "error_term"
            functions[controller]["gain"] = "gain"
            functions[controller]["output"] = "output"
            functions[controller]["param_pefix"] = controller.split("_")[0]

            data_structures[controller] = dict()
            # looping through the values of keys of functions[controller]

            for key in functions[controller]:
                if key == "name":
                    data_structures[controller][functions[controller][key]] = dict()

            for key in functions[controller]:
                print(key)
                # if key != "name":
                #     data_structures[controller][functions[controller][key]]["name"] = functions[controller][key]
                #     data_structures[controller][functions[controller][key]]["d_type"] = "float"
                #     data_structures[controller][functions[controller][key]]["init"] = 0.0
                #     data_structures[controller][functions[controller][key]]["is_pointer"] = False

        return {"functions": functions, "data_structures": data_structures}
