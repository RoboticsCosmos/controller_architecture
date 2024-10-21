import json
import glob

def check_id_references(jsonld):
    """
    check if all @id values are referenced in other keys in the JSON-LD document
    """
    id_references = dict()
    for item in jsonld["@graph"]:
        if "@id" in item:
            id_value = item["@id"]
            id_references[id_value] = list()
        for item in jsonld["@graph"]:
            for key, value in item.items():
                if key != "@id":
                    # check if value is a list
                    if isinstance(value, list):
                        for v in value:
                            if v == id_value:
                                id_references[id_value].append(key)
                    elif value == id_value:
                        id_references[id_value].append(key)

    unused_ids = []
    for id_value, references in id_references.items():
        if len(references) <1:
            unused_ids.append(id_value)
    
    return unused_ids

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
    directory_path = "/home/sawantk/freddy_ws/controller_architecture_thesis/controller-models/uc1/"
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
    # file_path = "/home/sawantk/freddy_ws/controller_architecture_thesis/controller-models/adaptive_pid_controller.jsonld"
    # with open(file_path, "r") as file:
    #     jsonld_data = json.load(file)
    #######################################################


    unused_ids = check_id_references(jsonld_data)

    print("# Checking for unused @id values in the JSON-LD document...")
    if unused_ids:
        print("The following @id values are not referenced in other keys in the JSON-LD document:")
        for unused_id in unused_ids:
            print(unused_id)
    else:
        print("All @id values are referenced in other keys in the JSON-LD document.")

    print("\n# Checking for availability of all @id values in the JSON-LD document...")
    unavailable_ids = check_availability_of_all_ids(jsonld_data)
    if unavailable_ids:
        print("The following URI shorthands are not available in the JSON-LD document:")
        for unavailable_id in unavailable_ids:
            print(unavailable_id)
    else:
        print("All shorthand URI values are available in the JSON-LD document.")

if __name__ == "__main__":
    main()
