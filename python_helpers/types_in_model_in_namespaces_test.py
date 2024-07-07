import json
import glob


def get_unique_types(jsonld):
    """
    check if all @id values are referenced in other keys in the JSON-LD document
    """
    unique_types = set()
    id_references = dict()
    for item in jsonld["@graph"]:
        for item in jsonld["@graph"]:
            for key, value in item.items():
                if key == "@type":
                    # check if value is a list
                    if isinstance(value, list):
                        for v in value:
                            unique_types.add(v)
                    else:
                        unique_types.add(value)

    return unique_types


def check_availability_of_all_ids(jsonld):
    """
    check if all referred @id values are available in the JSON-LD document
    """
    referred_ids = set()
    for item in jsonld["@graph"]:
        for key, value in item.items():
            if key != "@id":
                if isinstance(value, list):
                    for v in value:
                        if isinstance(v, str) and v.startswith("adap_ctr_uc1:"):
                            referred_ids.add(v)
                else:
                    if isinstance(value, str) and value.startswith("adap_ctr_uc1:"):
                        referred_ids.add(value)

    available_ids = set()
    for item in jsonld["@graph"]:
        if "@id" in item:
            available_ids.add(item["@id"])

    unavailable_ids = []
    for ids in referred_ids:
        if ids not in available_ids:
            unavailable_ids.append(ids)

    return unavailable_ids


def main():
    #######################################################
    """
    multiple jsonld check:
    """
    directory_path = (
        "/home/melody-u18/Desktop/Thesis/controller_architecture/controller-models/uc1/"
    )
    merged_graph = []
    file_paths = glob.glob(f"{directory_path}*.jsonld")
    for file_path in file_paths:
        with open(file_path, "r") as file:
            jsonld_data_indiv = json.load(file)
            graph_data = jsonld_data_indiv.get("@graph", [])
            merged_graph.extend(graph_data)

    jsonld_data = {"@graph": merged_graph}

    #######################################################
    """
    single jsonld check:
    """
    # file_path = "/home/melody-u18/Desktop/Thesis/controller_architecture/controller-models/adaptive_pid_controller.jsonld"
    # with open(file_path, "r") as file:
    #     jsonld_data = json.load(file)
    #######################################################

    unique_types_list = get_unique_types(jsonld_data)
    print("unique_types in the graph of the model: \n", unique_types_list)

    # loop through each unique types and check if it exists in the namespaces.py file. If not, then add it to not_found_types list
    not_found_types = []
    namespace_file = "/home/melody-u18/Desktop/Thesis/controller_architecture/synthesis/namespaces.py"
    for unique_type in unique_types_list:
        found = False
        with open(namespace_file, "r") as file:
            for line in file:
                if unique_type in line:
                    found = True
                    break
        if not found:
            not_found_types.append(unique_type)

    print("\ntypes from models that are not found in the namespaces.py: \n", not_found_types)


if __name__ == "__main__":
    main()
