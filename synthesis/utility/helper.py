import rdflib


def get_from_container(
    subject_node, predicate_value, graph, return_just_id_after_hash=False
):
    objects = []
    container_type = None
    for obj in graph.objects(subject_node, predicate_value):
        # Check if the object is a list
        if (obj, rdflib.RDF.first, None) in graph:
            container_type = "list"
            # Start traversing the list
            current_node = obj
            while current_node != rdflib.RDF.nil:
                # Get the value of the current node
                value = graph.value(current_node, rdflib.RDF.first)
                objects.append(value)
                # Move to the next node in the list
                current_node = graph.value(current_node, rdflib.RDF.rest)
        else:
            container_type = "set"
            objects.append(obj)
    if return_just_id_after_hash:
        objects = [graph.compute_qname(obj)[-1] for obj in objects]
    return objects, container_type


# with the help of chatgpt
def deep_update_dictionary(main_dict1, sub_dict):
    for k, v in sub_dict.items():
        if isinstance(v, dict) and k in main_dict1:
            deep_update_dictionary(sub_dict[k], v)
        else:
            main_dict1[k] = v
